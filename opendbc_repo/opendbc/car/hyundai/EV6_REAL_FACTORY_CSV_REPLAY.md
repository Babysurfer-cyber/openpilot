# 2022 EV6 순정 자동주차 CAN-FD 실제 로그 재생 검증 (읽기 전용)

## 테스트 조건

- 자료: 기존 사용자가 제공한 `extract_6.csv` (좌), `extract_7.csv` (우).
- 기록은 차량 CAN 데이터의 **기존 로그**다. 이 시험에서 차량/ECU에 통신을 연결하거나 새 명령을 송신하지 않았다.
- 원본 CSV 전체(약 1.17MB/1.15MB)는 PR에 포함하지 않았다.
- 비교 원작: `ajouatom/openpilot`의 `joongyu01/carrot-wip-dev`.
- 재생기: `ev6_spas_factory_replay.py` (오프라인, CAN/UDS/소켓 TX 없음).
- 기존 계획기: `ev6_spas_control.py`, 수신 전용 `passive_spas2_snapshot`.
- `bus=0`인 0x165 / 0x16A / 0x413만 OEM 관찰 근거로 사용;
  반환/복제된 2/128/130 소스는 독립 ECU 송신으로 계산하지 않음.
- SPAS1/2 길이·CRC16/XMODEM, 증가하는 카운터, 방향명령 onset 및
  실제 0x413 램프 OFF→ON 피드백을 검사한다.

## 실제 재생 결과

| 지표 | 좌측 로그(6) | 우측 로그(7) |
| --- | ---: | ---: |
| 순정 bus0 SPAS1 0x165 | 3,103 | 3,066 |
| 순정 bus0 SPAS2 0x16A | 3,104 | 3,066 |
| CRC 오류 | 0 | 0 |
| SPAS1 / SPAS2 counter 건너뜀 | 각 1회 | 각 1회 |
| counter 건너뜀 시각 및 두 프레임 간 간격 | 1.2243s / 42.3ms | 4.4467s / 33.0ms |
| 0→방향 명령 첫 시각 | 16.8990s, 값 3 | 20.3122s, 값 4 |
| 첫 0x413 램프 OFF→ON | 16.9284s | 20.3224s |
| 순정 명령→램프 피드백 | 29.4ms | 10.2ms |
| 방향 명령 시작 프레임 byte7, byte18 | 0x01, 0x40 | 0x01, 0x40 |
| **명령 유지 중 byte7/18 조합의 종류** | **8종** | **8종** |
| 정상 수신 전용 계획기 snapshot | 3,049 | 3,021 |
| bus0 0x7B1/0x7B9 기록 | 0 | 0 |

CRC 오류가 없다는 것은 기록된 SPAS frame checksum이 일치한다는
뜻이지, 모든 프레임 수신이 완전했다는 증거는 아니다. 두 기록 모두
0x165/0x16A counter가 1회씩 건너뛰었다.

## 원작 포팅에 주는 결론

1. `spas.py:frame()`의 대부분 바이트 0-fill SPAS2 프레임은
   실제 EV6 방향 명령 시작 프레임과 다르다. 특히 factory-on
   raw byte7=0x01, byte18=0x40이다.
2. 명령이 유지되는 동안 byte7/18 조합이 **8종 이상 변할 수 있으므로**
   `0x01/0x40`을 전체 지속시간 동안 하드코딩하는 것 역시 순정
   작동을 재현하지 못한다.
3. `hyundaicanfd.py:create_spas_messages()`의 OEM template
   복사는 더 많은 비트를 보존할 수 있지만, idle 상태의 템플릿을
   그대로 재사용해 방향명령만 덮어쓰는 것이 순정의 상태 전환과
   같다는 증거는 없다. 관련 바이트의 목적/인과는 여전히 미상.
4. 원작의 엄격한 `safety_hyundai_spas.h` TX 검사는 해당 byte7/18의
   비제로 factory-on payload를 그대로 승인하는 경로가 아니다.
   panda 예외를 열거나 우회해서 차량에 실험하는 조치로 해결하지 않는다.
5. 기존 0x7B1/0x7B9 진단 세션 시도는 **이 두 로그에 없다**.
   실제 ECU 인계 승인·순정 SPAS 송신 정지·복구를 입증하지 못한다.

## 실행 방법 (컴퓨터의 **저장된 CSV**에서만)

~~~bash
python opendbc_repo/opendbc/car/hyundai/ev6_spas_factory_replay.py \
  --left /path/to/extract_6.csv \
  --right /path/to/extract_7.csv \
  --output factory_replay_report.json

python -m unittest discover -s opendbc_repo/opendbc/car/hyundai \
  -p 'test_ev6_spas*.py' -v
~~~

현재 로컬에서 기존 계획기 단위 테스트 39개와 신규 재생기
회귀시험 10개를 **49/49 PASS**했다. 실제 두 CSV에서도 위 수치를
확인했고 `py_compile`와 CAN 송신 API 부재 AST 점검도 통과했다.
GitHub Actions의 CI 결과는 별개로 확인해야 한다.

**한계와 결론:** 이 도구는 순정 로그의 명령과 램프 피드백 사이의
시간 상관관계만 확인한다. 외부 장치가 같은 명령을 송신했을 때
점등한다거나, 원작의 무응답 fallback이 안전하다거나,
`0x7B1` ECU 제어권 전환에 응답한다는 증거가 전혀 아니다.
CAN 컨트롤러 error/bus-off나 ECU 출처를 독립 인증하는 기능도 없다.

**PR #47 Draft / NO-GO:** 운행 차량의 CAN/UDS 송신부 활성화,
panda 허용 목록 완화, 주행 브랜치 병합 및 실차 적용 금지.
