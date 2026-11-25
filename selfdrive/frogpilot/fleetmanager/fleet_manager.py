#!/usr/bin/env python3
# otisserv - Copyright (c) 2019-, Rick Lan, dragonpilot community, and a number of other of contributors.
# Fleet Manager - [actuallylemoncurd](https://github.com/actuallylemoncurd), [AlexandreSato](https://github.com/alexandreSato),
# [ntegan1](https://github.com/ntegan1), [royjr](https://github.com/royjr), and [sunnyhaibin] (https://github.com/sunnypilot)
# Almost everything else - ChatGPT
# dirty PR pusher - mike8643
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
import os
import re
import secrets
import subprocess
import threading
from concurrent.futures import ThreadPoolExecutor

from flask import Flask, jsonify, render_template, Response, request, send_from_directory, redirect, url_for, abort
from openpilot.common.realtime import set_core_affinity
import openpilot.selfdrive.frogpilot.fleetmanager.helpers as fleet
from openpilot.system.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog
import traceback
from ftplib import FTP

from openpilot.common.params import Params
from cereal import log, messaging
from cereal import log, messaging
import time
from functools import wraps
from openpilot.opendbc_repo.opendbc.car.interfaces import CarInterfaceBase
from openpilot.opendbc_repo.opendbc.car.values import PLATFORMS

# Initialize messaging
sm = messaging.SubMaster(['carState'])
tempseg = -1
temproute = "None"

app = Flask(__name__)

# Preview generation is bounded and never blocks the Dashcam page response.
PREVIEW_EXECUTOR = ThreadPoolExecutor(max_workers=1, thread_name_prefix="fleet-preview")
PREVIEW_LOCK = threading.Lock()
PREVIEW_PENDING = set()
PREVIEW_RETRY_AFTER = {}
PREVIEW_QUEUE_LIMIT = 8
PREVIEW_FAILURE_COOLDOWN = 180.0
FOOTAGE_PAGE_SIZE = 20
# Older/newer loggers may use either calendar timestamps or hex route IDs.
# Screenshot example: 00000d3f--5a12c700c9 (8 hex digits + "--" + 10 hex digits).
# Keep strict, path-safe IDs; never allow slashes or parent-directory traversal.
ROUTE_PATTERN = re.compile(r"^(?:\d{4}-\d{2}-\d{2}--\d{2}-\d{2}-\d{2}|[0-9a-fA-F]{8}--[0-9a-fA-F]{10})$")
SEGMENT_PATTERN = re.compile(ROUTE_PATTERN.pattern[:-1] + r"--\d+$")


def _create_preview(segment):
  segment_dir = os.path.join(Paths.log_root(), segment)
  source = os.path.join(segment_dir, "qcamera.ts")
  preview = os.path.join(segment_dir, "preview.gif")
  temporary = os.path.join(segment_dir, "preview.part.gif")
  success = False

  try:
    if not os.path.isfile(source):
      raise FileNotFoundError(source)
    # Seek before decoding; bound CPU usage and process duration.
    subprocess.run(
      ["ffmpeg", "-hide_banner", "-loglevel", "error", "-threads", "1",
       "-i", source, "-ss", "5", "-frames:v", "1", "-y", temporary],
      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10, check=True,
    )
    if not os.path.isfile(temporary) or os.path.getsize(temporary) == 0:
      raise RuntimeError("ffmpeg did not produce a preview")
    os.replace(temporary, preview)
    success = True
  except (OSError, subprocess.SubprocessError, RuntimeError) as exc:
    cloudlog.warning(f"Fleet preview failed for {segment}: {exc}")
  finally:
    if os.path.exists(temporary):
      try:
        os.remove(temporary)
      except OSError:
        pass
    with PREVIEW_LOCK:
      PREVIEW_PENDING.discard(segment)
      if success:
        PREVIEW_RETRY_AFTER.pop(segment, None)
      else:
        PREVIEW_RETRY_AFTER[segment] = time.monotonic() + PREVIEW_FAILURE_COOLDOWN


@app.route("/preview-thumbnail/<segment>")
def preview_thumbnail(segment):
  if not SEGMENT_PATTERN.fullmatch(segment):
    abort(404)

  preview_dir = os.path.join(Paths.log_root(), segment)
  preview_name = "preview.gif"
  preview_path = os.path.join(preview_dir, preview_name)

  if os.path.isfile(preview_path) and os.path.getsize(preview_path) > 0:
    return send_from_directory(preview_dir, preview_name, mimetype="image/gif", max_age=300)

  if not os.path.isfile(os.path.join(preview_dir, "qcamera.ts")):
    return jsonify({"status": "missing_recording"}), 404

  with PREVIEW_LOCK:
    now = time.monotonic()
    if now < PREVIEW_RETRY_AFTER.get(segment, 0):
      return jsonify({"status": "failed_retry_later"}), 503
    if segment not in PREVIEW_PENDING and len(PREVIEW_PENDING) < PREVIEW_QUEUE_LIMIT:
      PREVIEW_PENDING.add(segment)
      try:
        PREVIEW_EXECUTOR.submit(_create_preview, segment)
      except RuntimeError:
        PREVIEW_PENDING.discard(segment)
        return jsonify({"status": "unavailable"}), 503

  return jsonify({"status": "pending"}), 202

@app.route("/")
def home_page():
  return render_template("index.html")

@app.errorhandler(500)
def internal_error(exception):
  print('500 error caught')
  tberror = traceback.format_exc()
  return render_template("error.html", error=tberror)

@app.route("/footage/full/<cameratype>/<route>")
def full(cameratype, route):
  chunk_size = 1024 * 512  
  file_name = cameratype + (".ts" if cameratype == "qcamera" else ".hevc")
  vidlist = "|".join(Paths.log_root() + "/" + segment + "/" + file_name for segment in fleet.segments_in_route(route))
  
  def generate_buffered_stream():
    with fleet.ffmpeg_mp4_concat_wrap_process_builder(vidlist, cameratype, chunk_size) as process:
      for chunk in iter(lambda: process.stdout.read(chunk_size), b""):
        yield bytes(chunk)
  return Response(generate_buffered_stream(), status=200, mimetype='video/mp4')

@app.route("/footage/full/rlog/<route>/<segment>")
def download_rlog(route, segment):
  file_name = Paths.log_root() + route + "--" + segment + "/"
  print("download_route=", route, file_name, segment)
  return send_from_directory(file_name, "rlog.zst", as_attachment=True)

@app.route("/footage/full/qcamera/<route>/<segment>")
def download_qcamera(route, segment):
  file_name = Paths.log_root() + route + "--" + segment + "/"
  print("download_route=", route, file_name, segment)
  return send_from_directory(file_name, "qcamera.ts", as_attachment=True)

@app.route("/footage/full/fcamera/<route>/<segment>")
def download_fcamera(route, segment):
  file_name = Paths.log_root() + route + "--" + segment + "/"
  print("download_route=", route, file_name, segment)
  return send_from_directory(file_name, "fcamera.hevc", as_attachment=True)

@app.route("/footage/full/dcamera/<route>/<segment>")
def download_dcamera(route, segment):
  file_name = Paths.log_root() + route + "--" + segment + "/"
  print("download_route=", route, file_name, segment)
  return send_from_directory(file_name, "dcamera.hevc", as_attachment=True)
  
@app.route("/footage/full/ecamera/<route>/<segment>")
def download_ecamera(route, segment):
  file_name = Paths.log_root() + route + "--" + segment + "/"
  print("download_route=", route, file_name, segment)
  return send_from_directory(file_name, "ecamera.hevc", as_attachment=True)
        
def upload_folder_to_ftp(local_folder, directory, remote_path):
    from tqdm import tqdm
    ftp_server = "shind0.synology.me"
    ftp_port = 8021
    ftp_username = "carrotpilot"
    ftp_password = "Ekdrmsvkdlffjt7710"
    ftp = FTP()
    ftp.connect(ftp_server, ftp_port)
    ftp.login(ftp_username, ftp_password)

    ftp.cwd("routes")

    try:
        def create_path(path):
            try:
                ftp.mkd(path)
            except:
                pass
            ftp.cwd(path)

        for part in [directory, remote_path]:
            create_path(part)

        files = []
        for root, _, filenames in os.walk(local_folder):
            for filename in filenames:
                if filename in ['rlog.zst', 'qcamera.ts']:
                    files.append(os.path.join(root, filename))

        with tqdm(total=len(files), desc="Uploading Files", unit="file") as pbar:
            for local_file in files:
                filename = os.path.basename(local_file)
                try:
                    with open(local_file, 'rb') as f:
                        ftp.storbinary(f'STOR {filename}', f)
                    pbar.update(1)
                except Exception as e:
                    print(f"Failed to upload {local_file}: {e}")

        ftp.quit()
        return True
    except Exception as e:
        print(f"FTP Upload Error: {e}")
        return False
        

@app.route("/folder-info")
def get_folder_info():
    path = request.args.get('path')
    if not path or not os.path.exists(path):
        return jsonify({'error': 'Folder not found'}), 404
    
    try:
        folder_name = os.path.basename(path)
        seg_num = int(folder_name.split('--')[2])
        stat_info = os.stat(path)
        created_time = stat_info.st_ctime
        if seg_num == 0:
            try:
                base_name = '--'.join(folder_name.split('--')[:2])
                seg1_path = os.path.join(os.path.dirname(path), f"{base_name}--1")
                
                if os.path.exists(seg1_path):
                    seg1_stat = os.stat(seg1_path)
                    created_time = seg1_stat.st_ctime - 60
            except Exception as e:
                print(f"Error calculating time for segment 0: {e}")
        formatted_date = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(created_time))
        total_size = 0
        for dirpath, dirnames, filenames in os.walk(path):
            for f in filenames:
                fp = os.path.join(dirpath, f)
                try:
                    total_size += os.path.getsize(fp)
                except OSError:
                    continue
        
        return jsonify({
            'created_date': formatted_date,
            'size': total_size,
            'status': 'success'
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 500
        
@app.route("/folder-date")
def get_folder_date():
    path = request.args.get('path')
    subtract_minutes = int(request.args.get('subtract_minutes', 0))
    
    if not path or not os.path.exists(path):
        return jsonify({'error': 'Folder not found'}), 404
    
    try:
        stat_info = os.stat(path)
        created_time = stat_info.st_ctime
        
        if subtract_minutes > 0:
            created_time -= subtract_minutes * 60
        
        formatted_date = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(created_time))
        
        return jsonify({
            'date': formatted_date,
            'status': 'success'
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route("/folder-exists")
def check_folder_exists():
    path = request.args.get('path')
    exists = os.path.exists(path) if path else False
    return jsonify({'exists': exists})

@app.route("/footage/full/upload_carrot/<route>/<segment>", methods=['POST'])
def upload_carrot(route, segment):
    global tempseg
    global temproute
    if tempseg != segment or temproute != route:
        local_folder = os.path.join(Paths.log_root(), f"{route}--{segment}")
        if not os.path.isdir(local_folder):
            abort(404, "Folder not found")
        car_selected = Params().get("CarName", "none")
        dongle_id = Params().get("DongleId", "unknown")
        directory = f"{car_selected} {dongle_id}"
        success = upload_folder_to_ftp(local_folder, directory, f"{route}--{segment}")
        if success:
            temproute = route
            tempseg = segment
            return "All files uploaded successfully", 200
        else:
            return "Failed to upload files", 500
    else:
        return "Segment already uploaded", 200
        
@app.template_filter('datetimeformat')
def datetimeformat_filter(filename):
    try:
        date_part = filename[:8]
        time_part = filename[9:15]
        
        year = date_part[:4]
        month = date_part[4:6]
        day = date_part[6:8]
        
        hour = time_part[:2]
        minute = time_part[2:4]
        
        return f"{year}년 {month}월 {day}일 {hour}시 {minute}분"
    except:
        return filename
        
@app.route("/file-size")
def get_file_size():
    path = request.args.get('path')
    try:
        size = os.path.getsize(path)
        return jsonify({'size': size, 'status': 'success'})
    except Exception as e:
        return jsonify({'size': 0, 'status': str(e)}), 404

@app.route("/footage/<cameratype>/<segment>")
def fcamera(cameratype, segment):
  if cameratype not in ("qcamera", "fcamera", "dcamera", "ecamera"):
    abort(404)
  if not SEGMENT_PATTERN.fullmatch(segment) or not fleet.is_valid_segment(segment):
    abort(404)
  file_name = os.path.join(Paths.log_root(), segment, cameratype + (".ts" if cameratype == "qcamera" else ".hevc"))
  if not os.path.isfile(file_name) or os.path.getsize(file_name) == 0:
    abort(404)
  process = fleet.ffmpeg_mp4_wrap_process_builder(file_name)

  def stream_video():
    try:
      while True:
        chunk = process.stdout.read(65536)
        if not chunk:
          break
        yield chunk
    finally:
      if process.stdout:
        process.stdout.close()
      if process.poll() is None:
        process.terminate()
        try:
          process.wait(timeout=2)
        except subprocess.TimeoutExpired:
          process.kill()
          process.wait()

  return Response(stream_video(), status=200, mimetype="video/mp4")


@app.route("/footage/<route>")
def route(route):
    if not ROUTE_PATTERN.fullmatch(route):
        abort(404)
    query_params = request.query_string.decode("utf-8").split(",")
    query_segment = query_params[0] if query_params and query_params[0].isdigit() else "0"
    query_type = query_params[1] if len(query_params) >= 2 and query_params[1] in ("qcamera", "fcamera", "dcamera", "ecamera") else "qcamera"

    segments = fleet.segments_in_route(route)
    if not segments:
        return render_template("error.html", error="No recording segments are stored for this route")

    file_name = query_type + (".ts" if query_type == "qcamera" else ".hevc")
    selected = f"{route}--{query_segment}"
    if selected not in segments or not os.path.isfile(os.path.join(Paths.log_root(), selected, file_name)):
        playable = [segment for segment in segments
                    if os.path.isfile(os.path.join(Paths.log_root(), segment, file_name))]
        if not playable:
            return render_template("error.html", error="No playable video for this camera; only logs may remain")
        query_segment = playable[0].rsplit("--", 1)[-1]

    links = [f'<a href="{route}?{segment.rsplit("--", 1)[-1]},{query_type}">{segment}</a>'
             for segment in segments]
    return render_template("route.html", route=route, query_type=query_type,
                           links="<br>".join(links),
                           segments=",".join(f"'{segment}'" for segment in segments),
                           query_segment=query_segment)

@app.route("/footage/")
@app.route("/footage")
def footage():
  # Read directory names once. Segment 0 can be absent after retention cleanup.
  groups = {}
  for item in fleet.all_segment_names():
    groups.setdefault(item.time_str, []).append(item.segment_num)
  names = sorted(groups, reverse=True)
  total_routes = len(names)
  total_pages = max(1, (total_routes + FOOTAGE_PAGE_SIZE - 1) // FOOTAGE_PAGE_SIZE)
  page = request.args.get("page", default=1, type=int) or 1
  page = min(max(1, page), total_pages)
  page_names = names[(page - 1) * FOOTAGE_PAGE_SIZE:page * FOOTAGE_PAGE_SIZE]
  rows = []
  for name in page_names:
    numbers = sorted(groups[name])
    segment = next(
      (f"{name}--{n}" for n in numbers
       if os.path.isfile(os.path.join(Paths.log_root(), f"{name}--{n}", "qcamera.ts"))), None
    )
    try:
      date_folder = os.path.join(Paths.log_root(), f"{name}--{numbers[0]}")
      date_text = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(os.stat(date_folder).st_ctime))
    except (IndexError, OSError):
      date_text = "저장된 날짜 정보 없음"
    rows.append({"name": name, "segment": segment, "date": date_text})
  return render_template("footage.html", routes=rows, page=page,
                         total_pages=total_pages, total_routes=total_routes)

@app.route("/preserved/")
@app.route("/preserved")
def preserved():
  query_type = "qcamera"
  route_paths = []
  gifs = []
  segments = fleet.preserved_routes()
  for segment in segments:
    input_path = Paths.log_root() + segment + "/qcamera.ts"
    output_path = Paths.log_root() + segment + "/preview.gif"
    fleet.video_to_img(input_path, output_path)
    split_segment = segment.split("--")
    route_paths.append(f"{split_segment[0]}--{split_segment[1]}?{split_segment[2]},{query_type}")
    gif_path = segment + "/preview.gif"
    gifs.append(gif_path)

  zipped = zip(route_paths, gifs, segments, strict=False)
  return render_template("preserved.html", zipped=zipped)

@app.route("/screenrecords/")
@app.route("/screenrecords")
def screenrecords():
  rows = fleet.list_file(fleet.SCREENRECORD_PATH)
  if not rows:
    return render_template("error.html", error="no screenrecords found")
  files_with_size = []
  for file in rows:
    file_path = os.path.join(fleet.SCREENRECORD_PATH, file)
    size_bytes = os.path.getsize(file_path) if os.path.exists(file_path) else 0
    files_with_size.append((file, size_bytes))
  return render_template("screenrecords.html", rows=files_with_size, clip=rows[0])


@app.route("/screenrecords/<clip>")
def screenrecord(clip):
  rows = fleet.list_file(fleet.SCREENRECORD_PATH)
  files_with_size = []
  for file in rows:
    file_path = os.path.join(fleet.SCREENRECORD_PATH, file)
    size_bytes = os.path.getsize(file_path) if os.path.exists(file_path) else 0
    files_with_size.append((file, size_bytes))
  return render_template("screenrecords.html", rows=files_with_size, clip=clip)


@app.route("/screenrecords/play/pipe/<file>")
def videoscreenrecord(file):
  file_name = fleet.SCREENRECORD_PATH + file
  return Response(fleet.ffplay_mp4_wrap_process_builder(file_name).stdout.read(), status=200, mimetype='video/mp4')


@app.route("/screenrecords/download/<clip>")
def download_file(clip):
  return send_from_directory(fleet.SCREENRECORD_PATH, clip, as_attachment=True)


@app.route("/about")
def about():
  return render_template("about.html")


@app.route("/error_logs")
def error_logs():
  rows = fleet.list_file(fleet.ERROR_LOGS_PATH)
  if not rows:
    return render_template("error.html", error="no error logs found at:<br><br>" + fleet.ERROR_LOGS_PATH)
  return render_template("error_logs.html", rows=rows)


@app.route("/error_logs/<file_name>")
def open_error_log(file_name):
  f = open(fleet.ERROR_LOGS_PATH + file_name)
  error = f.read()
  return render_template("error_log.html", file_name=file_name, file_content=error)

@app.route("/addr_input", methods=['GET', 'POST'])
def addr_input():
  preload = fleet.preload_favs()
  SearchInput = fleet.get_SearchInput()
  token = fleet.get_public_token()
  s_token = fleet.get_app_token()
  gmap_key = fleet.get_gmap_key()
  PrimeType = fleet.get_PrimeType()
  lon = 0.0
  lat = 0.0
  print(f"Request method: {request.method}, SearchInput: {SearchInput}, token: {token}, s_token: {s_token}, gmap_key: {gmap_key}, PrimeType: {PrimeType}")
  if request.method == 'POST':
    valid_addr = False
    postvars = request.form.to_dict()
    addr, lon, lat, valid_addr, token = fleet.parse_addr(postvars, lon, lat, valid_addr, token)
    if not valid_addr:
      # If address is not found, try searching
      postvars = request.form.to_dict()
      addr = request.form.get('addr_val')
      addr, lon, lat, valid_addr, token = fleet.search_addr(postvars, lon, lat, valid_addr, token)
    if valid_addr:
      # If a valid address is found, redirect to nav_confirmation
      return redirect(url_for('nav_confirmation', addr=addr, lon=lon, lat=lat))
    else:
      return render_template("error.html")
  #elif PrimeType != 0:
  #  return render_template("prime.html")
  # amap stuff
  elif SearchInput == 1:
    amap_key, amap_key_2 = fleet.get_amap_key()
    if amap_key == "" or amap_key is None or amap_key_2 == "" or amap_key_2 is None:
      return redirect(url_for('amap_key_input'))
    elif token == "" or token is None:
      return redirect(url_for('public_token_input'))
    elif s_token == "" or s_token is None:
      return redirect(url_for('app_token_input'))
    else:
      return redirect(url_for('amap_addr_input'))
  elif False: #fleet.get_nav_active(): # carrot: 그냥지움... 이것때문에 토큰을 안물어보는듯...
    if SearchInput == 2:
      return render_template("nonprime.html",
                             gmap_key=gmap_key, lon=lon, lat=lat,
                             home=preload[0],work=preload[1], fav1=preload[2], fav2=preload[3], fav3=preload[4])
    else:
      return render_template("nonprime.html",
                             gmap_key=None, lon=None, lat=None,
                             home=preload[0], work=preload[1], fav1=preload[2], fav2=preload[3], fav3=preload[4])
  elif token == "" or token is None:
    return redirect(url_for('public_token_input'))
  elif s_token == "" or s_token is None:
    return redirect(url_for('app_token_input'))
  elif SearchInput == 2:
    lon, lat = fleet.get_last_lon_lat()
    if gmap_key == "" or gmap_key is None:
      return redirect(url_for('gmap_key_input'))
    else:
      return render_template("addr.html",
                             gmap_key=gmap_key, lon=lon, lat=lat,
                             home=preload[0], work=preload[1], fav1=preload[2], fav2=preload[3], fav3=preload[4])
  else:
      return render_template("addr.html",
                             gmap_key=None, lon=None, lat=None,
                             home=preload[0], work=preload[1], fav1=preload[2], fav2=preload[3], fav3=preload[4])

@app.route("/nav_confirmation", methods=['GET', 'POST'])
def nav_confirmation():
  token = fleet.get_public_token()
  lon = request.args.get('lon')
  lat = request.args.get('lat')
  addr = request.args.get('addr')
  if request.method == 'POST':
    postvars = request.form.to_dict()
    fleet.nav_confirmed(postvars)
    return redirect(url_for('addr_input'))
  else:
    return render_template("nav_confirmation.html", addr=addr, lon=lon, lat=lat, token=token)

@app.route("/public_token_input", methods=['GET', 'POST'])
def public_token_input():
  if request.method == 'POST':
    postvars = request.form.to_dict()
    fleet.public_token_input(postvars)
    return redirect(url_for('addr_input'))
  else:
    return render_template("public_token_input.html")

@app.route("/app_token_input", methods=['GET', 'POST'])
def app_token_input():
  if request.method == 'POST':
    postvars = request.form.to_dict()
    fleet.app_token_input(postvars)
    return redirect(url_for('addr_input'))
  else:
    return render_template("app_token_input.html")

@app.route("/gmap_key_input", methods=['GET', 'POST'])
def gmap_key_input():
  if request.method == 'POST':
    postvars = request.form.to_dict()
    fleet.gmap_key_input(postvars)
    return redirect(url_for('addr_input'))
  else:
    return render_template("gmap_key_input.html")

@app.route("/amap_key_input", methods=['GET', 'POST'])
def amap_key_input():
  if request.method == 'POST':
    postvars = request.form.to_dict()
    fleet.amap_key_input(postvars)
    return redirect(url_for('amap_addr_input'))
  else:
    return render_template("amap_key_input.html")

@app.route("/amap_addr_input", methods=['GET', 'POST'])
def amap_addr_input():
  if request.method == 'POST':
    postvars = request.form.to_dict()
    fleet.nav_confirmed(postvars)
    return redirect(url_for('amap_addr_input'))
  else:
    lon, lat = fleet.get_last_lon_lat()
    amap_key, amap_key_2 = fleet.get_amap_key()
    return render_template("amap_addr_input.html", lon=lon, lat=lat, amap_key=amap_key, amap_key_2=amap_key_2)

@app.route("/CurrentStep.json", methods=['GET'])
def find_CurrentStep():
  directory = "/data/openpilot/selfdrive/manager/"
  filename = "CurrentStep.json"
  return send_from_directory(directory, filename, as_attachment=True)

@app.route("/navdirections.json", methods=['GET'])
def find_nav_directions():
  directory = "/data/openpilot/selfdrive/manager/"
  filename = "navdirections.json"
  return send_from_directory(directory, filename, as_attachment=True)

@app.route("/locations", methods=['GET'])
def get_locations():
  data = fleet.get_locations()
  return Response(data, content_type="application/json")

@app.route("/set_destination", methods=['POST'])
def set_destination():
  valid_addr = False
  postvars = request.get_json()
  data, valid_addr = fleet.set_destination(postvars, valid_addr)
  if valid_addr:
    return Response('{"success": true}', content_type='application/json')
  else:
    return Response('{"success": false}', content_type='application/json')

@app.route("/navigation/<file_name>", methods=['GET'])
def find_navicon(file_name):
  directory = "/data/openpilot/selfdrive/assets/navigation/"
  return send_from_directory(directory, file_name, as_attachment=True)

@app.route("/previewgif/<path:file_path>", methods=['GET'])
def find_previewgif(file_path):
  directory = "/data/media/0/realdata/"
  return send_from_directory(directory, file_path, as_attachment=True)

@app.route("/tools", methods=['GET'])
def tools_route():
  return render_template("tools.html")

@app.route("/get_toggle_values", methods=['GET'])
def get_toggle_values_route():
  toggle_values = fleet.get_all_toggle_values()
  return jsonify(toggle_values)

@app.route("/store_toggle_values", methods=['POST'])
def store_toggle_values_route():
  try:
    updated_values = request.get_json()
    fleet.store_toggle_values(updated_values)
    return jsonify({"message": "Values updated successfully"}), 200
  except Exception as e:
    return jsonify({"error": "Failed to update values", "details": str(e)}), 400

@app.route("/carinfo")
def carinfo():
    try:
        params = Params()
        # 更新消息
        sm.update()

        # 获取车辆基本信息
        try:
            car_name = params.get("CarName")
            if car_name in PLATFORMS:
                platform = PLATFORMS[car_name]
                car_fingerprint = platform.config.platform_str
                car_specs = platform.config.specs
            else:
                car_fingerprint = "Unknown Fingerprint"
                car_specs = None
        except Exception as e:
            print(f"Failed to get vehicle basic info: {e}")
            car_name = "Unknown Model"
            car_fingerprint = "Unknown Fingerprint"
            car_specs = None

        # 获取车辆状态信息
        try:
            CS = sm['carState']

            # 基本状态判断
            is_car_started = CS.vEgo > 0.1
            is_car_engaged = CS.cruiseState.enabled

            # 构建基础信息
            car_info = {
                "Vehicle Status": {
                    "Running Status": "Moving" if is_car_started else "Stopped",
                    "Cruise System": "Enabled" if is_car_engaged else "Disabled",
                    "Current Speed": f"{CS.vEgo * 3.6:.1f} km/h",
                    "Engine RPM": f"{CS.engineRPM:.0f} RPM" if hasattr(CS, 'engineRPM') and CS.engineRPM > 0 else "Unknown",
                    "Gear Position": str(CS.gearShifter) if hasattr(CS, 'gearShifter') else "Unknown"
                },
                "Basic Information": {
                    "Car Model": car_name,
                    "Fingerprint": str(car_fingerprint),
                    "Weight": f"{car_specs.mass:.0f} kg" if car_specs and hasattr(car_specs, 'mass') else "Unknown",
                    "Wheelbase": f"{car_specs.wheelbase:.3f} m" if car_specs and hasattr(car_specs, 'wheelbase') else "Unknown",
                    "Steering Ratio": f"{car_specs.steerRatio:.1f}" if car_specs and hasattr(car_specs, 'steerRatio') else "Unknown"
                }
            }

            # 详细信息
            if is_car_started or is_car_engaged:
                car_info.update({
                    "Cruise Information": {
                        "Cruise Status": "On" if CS.cruiseState.enabled else "Off",
                        "Adaptive Cruise": "On" if CS.cruiseState.available else "Off",
                        "Set Speed": f"{CS.cruiseState.speed * 3.6:.1f} km/h" if CS.cruiseState.speed > 0 else "Not Set",
                        "Following Distance": str(CS.cruiseState.followDistance) if hasattr(CS.cruiseState, 'followDistance') else "Unknown"
                    },
                    "Wheel Speeds": {
                        "Front Left": f"{CS.wheelSpeeds.fl * 3.6:.1f} km/h",
                        "Front Right": f"{CS.wheelSpeeds.fr * 3.6:.1f} km/h",
                        "Rear Left": f"{CS.wheelSpeeds.rl * 3.6:.1f} km/h",
                        "Rear Right": f"{CS.wheelSpeeds.rr * 3.6:.1f} km/h"
                    },
                    "Steering System": {
                        "Steering Angle": f"{CS.steeringAngleDeg:.1f}°",
                        "Steering Torque": f"{CS.steeringTorque:.1f} Nm",
                        "Steering Rate": f"{CS.steeringRateDeg:.1f}°/s",
                        "Lane Departure": "Yes" if CS.leftBlinker or CS.rightBlinker else "No"
                    },
                    "Pedal Status": {
                        "Throttle Position": f"{CS.gas * 100:.1f}%",
                        "Brake Pressure": f"{CS.brake * 100:.1f}%",
                        "Gas Pedal": "Pressed" if CS.gasPressed else "Released",
                        "Brake Pedal": "Pressed" if CS.brakePressed else "Released"
                    },
                    "Safety Systems": {
                        "ESP Status": "Active" if CS.espDisabled else "Normal",
                        "ABS Status": "Active" if hasattr(CS, 'absActive') and CS.absActive else "Normal",
                        "Traction Control": "Active" if hasattr(CS, 'tcsActive') and CS.tcsActive else "Normal",
                        "Collision Warning": "Warning" if hasattr(CS, 'collisionWarning') and CS.collisionWarning else "Normal"
                    },
                    "Door Status": {
                        "Driver Door": "Open" if CS.doorOpen else "Closed",
                        "Passenger Door": "Open" if hasattr(CS, 'passengerDoorOpen') and CS.passengerDoorOpen else "Closed",
                        "Trunk": "Open" if hasattr(CS, 'trunkOpen') and CS.trunkOpen else "Closed",
                        "Hood": "Open" if hasattr(CS, 'hoodOpen') and CS.hoodOpen else "Closed",
                        "Seatbelt": "Unbuckled" if CS.seatbeltUnlatched else "Buckled"
                    },
                    "Light Status": {
                        "Left Turn Signal": "On" if CS.leftBlinker else "Off",
                        "Right Turn Signal": "On" if CS.rightBlinker else "Off",
                        "High Beam": "On" if CS.genericToggle else "Off",
                        "Low Beam": "On" if hasattr(CS, 'lowBeamOn') and CS.lowBeamOn else "Off"
                    },
                    "Blind Spot Monitor": {
                        "Left Side": "Vehicle Detected" if CS.leftBlindspot else "Clear",
                        "Right Side": "Vehicle Detected" if CS.rightBlindspot else "Clear"
                    }
                })

                # 添加可选的其他信息
                other_info = {}
                if hasattr(CS, 'outsideTemp'):
                    other_info["Outside Temperature"] = f"{CS.outsideTemp:.1f}°C"
                if hasattr(CS, 'fuelGauge'):
                    other_info["Range"] = f"{CS.fuelGauge:.1f}km"
                if hasattr(CS, 'odometer'):
                    other_info["Odometer"] = f"{CS.odometer:.1f}km"
                if hasattr(CS, 'instantFuelConsumption'):
                    other_info["Instant Fuel Consumption"] = f"{CS.instantFuelConsumption:.1f}L/100km"

                if other_info:
                    car_info["Other Information"] = other_info

        except Exception as e:
            print(f"Error getting vehicle state info: {str(e)}")
            traceback.print_exc()
            car_info = {
                "Basic Information": {
                    "Car Model": car_name,
                    "Fingerprint": str(car_fingerprint)
                },
                "Status": "Unable to get vehicle state information, please check if the vehicle is started"
            }

        return render_template("carinfo.html", car_info=car_info)

    except Exception as e:
        print(f"Error rendering carinfo page: {str(e)}")
        traceback.print_exc()
        return render_template("carinfo.html", car_info={"error": f"Error getting vehicle information: {str(e)}"})

def main():
  try:
    set_core_affinity([0, 1, 2, 3])
  except Exception:
    cloudlog.exception("fleet_manager: failed to set core affinity")
  app.secret_key = secrets.token_hex(32)
  app.run(host="0.0.0.0", port=8082)


if __name__ == '__main__':
  main()
