# 현대 CAN-FD 0x180 코너레이더 트랙

> 아래는 최초 0x180 이식 설명이다. 0x235 자동 감지와 원본 cut-in/cut-out·정지차
> 판정을 추가한 현재 동작과 설정은 [현대 레이더 궤적 처리](hyundai_radar_motion.md)를 따른다.

`ajouatom/openpilot`의 `carrot-wip` 커밋
`162da468cc4157f9b18daa4f6ac85438f51719ef`에서 0x180 코너레이더 물체
디코딩과 추적을 `ms-xy-radar3`에 이식했다.

## 사용 조건

- 현대/기아 CAN-FD 차량의 ACAN에서 `0x180`부터 `0x184`까지 **다섯 메시지가
  모두 32바이트**로 감지되어야 한다. 메시지마다 물체 두 개, 최대 열 개를 추적한다.
- 시작 설정의 **코너레이더 트랙**(`EnableCornerRadar`)을 켠 뒤 재시작한다.
  `EnableRadarTracks`와 독립적으로 켤 수 있다.
- 기존 전방 레이더와 SCC 설정은 그대로 사용한다. 코너레이더만 감지되는 경우에도
  트랙은 20Hz로 발행한다.

물체가 CAN 슬롯 사이를 이동하면 같은 추적 ID를 유지한다. 가까운 중복 물체만
합치며, 같은 센서 물체 번호를 쓰는 먼 물체와 번호가 재사용된 물체는 구분한다.
개별 메시지가 0.5초 넘게 수신되지 않으면 그 메시지의 물체를 제거한다.

## 추종 및 화면 표시

코너레이더 물체에는 `corner180` 출처를 붙여 전방 레이더 및 SCC와 구분한다.
옆에 붙어 종방향 거리가 0으로 잘린 물체는 `radarState`의 측면 목록에만 유지한다.
원본과 같이 도로 화면 투영은 종방향 거리 2.5m 초과 물체에 적용한다.
레이더 정보 표시(`ShowRadarInfo`)를 켜면 투영된 코너 트랙을 보라색 `C`로 구분한다.
전방 코너 물체는 이 브랜치의 기존 끼어들기 판정 조건을 통과해야 추종 후보가 된다.
주 비전 매칭, 기존 추종 대상 유지, 중앙 레이더 단독 추종에는 직접 사용하지 않는다.
기존 후측방 거리 기반 기능은 `EnableCornerRadar=2`에서 유지한다.

원본 브랜치의 별도 dPath 추종 시스템이나 0x235 및 미검증 0x430/0x440 디코더는
이번 이식에 포함하지 않았다.

## DBC 생성 및 검증

저장소 루트에서 실행한다.

```sh
PYTHONPATH="$PWD" python3 opendbc_repo/opendbc/dbc/generator/generator.py
```

생성 입력은 `generator/hyundai/hyundai_canfd_corner_radar_180.dbc`이며, 파서는
`opendbc_repo/opendbc/dbc/hyundai_canfd_corner_radar_180_generated.dbc`를 읽는다.
일반 SCons 빌드에서도 생성된다. 예전 입력 파일의 `_generated.dbc` 이름은
런타임에서 `_generated_generated.dbc`로 생성되므로 올바른 입력 이름으로 변경했다.

회귀 테스트는 CAN 바이트 디코딩, 기능 감지, 추적 ID, 메시지 소실, 발행 주기 및
추종 후보 제한을 확인한다. 실제 차량 로그 재생과 차량에서의 화면/주행 검증은 별도로 필요하다.
