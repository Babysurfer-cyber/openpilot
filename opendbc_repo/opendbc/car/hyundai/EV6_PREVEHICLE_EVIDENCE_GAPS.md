# EV6 방향지시등 실차 적용 전 필수 증거 점검 — 2026-11-25

이 문서는 **PR #47 (Draft / NO-GO)**의 독립적인 수신 전용
점검 결과다. 방향지시등을 켜는 명령을 차량에 보내지 않는다.

## 현재 실제 EV6 로그로 확인한 값

기존의 `extract_6.csv` (좌측) 및 `extract_7.csv` (우측)를
`ev6_spas_evidence_gate.py`로 분석했다. 원본 CSV는 이 PR에
업로드하지 않았으며 실험 중 어떤 차량 통신도 수행하지 않았다.

| 원본 버스 0 CAN ID | 좌측 | 우측 | 의미 |
| --- | ---: | ---: | --- |
| 0x165 | 3,103 | 3,066 | 순정 주차 SPAS1 |
| 0x16A | 3,104 | 3,066 | 순정 주차 SPAS2 |
| 0x3C1 | 309 | 306 | 방향지시등 레버 신호 |
| 0x413 | 479 | 457 | 좌·우 램프 상태 표본 |
| **0x35** | **0** | **0** | PR #46 기어 상태 확인에 필요 |
| **0x175** | **0** | **0** | PR #46 브레이크 상태 확인에 필요 |
| **0xA0** | **0** | **0** | PR #46 바퀴 속도 확인에 필요 |
| 0x7B1 | 0 | 0 | 외부 APRK ECU 진단 요청 |
| 0x7B9 | 0 | 0 | APRK ECU 응답 |

두 기록의 SPAS CRC 검사 오류 0건, 레버 CRC 오류 0건.
다만 **기어/브레이크/차륜속도 검증 프레임이 아예 추출되지 않았으므로**
PR #46의 독립 P단 안전 코드가 실제 사용자 EV6의 bus/gear
설정과 맞는지는 이 기록으로 검증할 수 없다.

**동시에 다음 항목은 여전히 미입증이다.**

- 실제 EV6 주차 ECU가 외부 0x7B1 요청에 ACK를 보내고
  0x165/0x16A 송신을 멈춘 뒤 안정적으로 복구하는지
- 순정 ECU 출처 및 CAN 분석기 오류/손실/버스오프 상태를
  별도 장비로 독립 확인했는지
- 사용자의 실제 EV6 panda safetyParam, 물리 통신 버스 구조,
  PR #46의 full libsafety 통합/빌드가 정상인지
- 별도 명령으로 방향지시등이 켜지면서 다른 ADAS 안전 기능에
  영향을 주지 않는지

현재 자동 점검기는 **양측 모두 NO_GO_FOR_VEHICLE_CAN_TRANSMIT**
이라고 표시한다. 이 결과는 증거의 누락 여부를 알려줄 뿐이며,
어떤 입력을 주더라도 실차 송신 허가를 발급하지 않는다.

## 장비 추가 구매 없이 가능한 다음 단계: 저장된 rlog 재분석

기존 콤마3의 **이미 기록된 rlog 파일**을 오픈파일럿이
설치된 개발 환경에서 `ev6_spas_rlog_extract.py`로 읽는다.
이 도구는 수신된 `can` 이벤트와 지정된 9개 CAN ID만 뽑아
네 개 CSV 칼럼(`t_sec,can_id,bus,payload_hex`)으로 출력한다.
`sendcan`, GPS, 카메라, 운전자 영상은 내보내지 않는다.

~~~bash
python opendbc_repo/opendbc/car/hyundai/ev6_spas_rlog_extract.py \
  --input '/local/saved/rlog.zst' \
  --output '/local/filtered_ev6_can.csv'

python opendbc_repo/opendbc/car/hyundai/ev6_spas_evidence_gate.py \
  --left '/local/left_capture.csv' \
  --right '/local/right_capture.csv' \
  --output '/local/evidence_gap_report.json'
~~~

CSV의 `bus=0/1/2`는 **수신 기록에 적힌 메타데이터**이다.
panda에서 버스 간 복제/포워딩된 프레임은 순정 ECU가
별도로 송신했다는 증거가 아니다. 특히 PR #46의 현재
C 검사기는 명시적으로 **bus 0** 원본 기어·브레이크·속도
검증을 요구하고, CAMERA_SCC가 아닌 HDA2 버스 구조에서는
그 RX 검사가 bus 1에 배정될 수 있어 차단될 수 있다.
버스가 다르다는 이유로 안전 정책을 완화하지 않는다.

`rlog`의 타임스탬프·프레임은 개인정보(영상 등)를 포함할 수 있다.
공유가 필요하면 원본보다 지정 CAN ID만 남긴 CSV를 우선 검토한다.
새 주행·CAN 송신·진단 요청·주차 ECU 비활성화를 하지 않는다.
**순정 상태에서 이미 확보한 로그의 오프라인 분석만 수행한다.**

## 수행한 소프트웨어 테스트와 실제 한계

기존 PR #47 Python 79개 + 새로운 증거 누락 판정 9개 +
저장된 rlog 필터/내보내기 8개 = **96/96 Python unittest PASS**.
`py_compile` 및 오프라인 Python 도구 6개의 CAN 송신
인터페이스 부재 AST 검사 통과. 이 체크는 개발용 정적 검사이며
하드웨어 안전 인증이나 전체 panda C 빌드가 아니다.

**추가 확보가 필요한 실제 데이터:** 선택된 이력 로그의 원본
0x35/0x175/0xA0가 어느 버스에 등장하는지, 각 실제 신호값,
정확한 safetyParam 및 panda RX checker 설정이다.
그 다음에도 ECU 인계/복구와 bus-off 없는 실제 안전성은
독립적인 격리 ECU/CAN-FD 벤치 없이는 증명할 수 없다.

**결론: 병합 금지.** PR #47은 순수 분석/모의 계획
Draft, 차량용 SPAS2 송신 미구현. PR #46의 실험용
0x3C1 C 정책도 실차 부착·주행 허가가 아니다.


## 2026-11-25 추가: 버스 0/1/2 증거 분리 및 개인정보 최소화

PR #46 실제 panda C 정책을 대조하여 중요한 버스 매핑 문제를 확인했다.

- 일반 HDA2 / non-CAMERA_SCC 구성에서는 \`HYUNDAI_CANFD_COMMON_RX_CHECKS(1)\`가
  적용될 수 있다. 그러나 PR #46 시험용 독립 주차·브레이크·차륜속도
  안전 게이트는 검증된 **bus 0**의 \`0x35/0x175/0xA0\`를 요구한다.
  원본 차량의 \`safetyParam\` 및 물리 버스 구성이 확인되지 않았으므로
  해당 게이트를 **완화하면 안 된다**.
- 최신 \`ev6_spas_evidence_gate.py\`는 원본 파일의 버스 0·1·2 CAN ID
  건수를 별도로 보존하고, 세 안전 입력 신호가 **한 버스에 함께 있는지**
  표시한다. bus1에서만 세 신호가 관찰되면
  \`HDA2_BUS1_RX_EVIDENCE_CANNOT_SATISFY_UNVERIFIED_BUS0_PANDA_GUARD\`
  항목을 추가한다. bus0/1/2의 부분 표본을 합쳐 정상으로 판단하지 않는다.
- 어느 버스에서 메시지 ID가 발견되어도 그 사실만으로 기어·브레이크·
  속도값의 정합성, CRC·최신성, 원본 ECU의 출처, 실제 panda
  RX checker 일치 여부를 증명하지 못한다. 그래서 **NO-GO를 유지**한다.
- 과거 \`extract_6.csv\` 및 \`extract_7.csv\`는 **모든 버스에서** 세
  안전 입력 CAN ID의 추출 건수가 0이다. 이는 처음의 CSV 추출 필터가
  생략했을 가능성을 포함하며, **전체 원본 rlog의 부재를 뜻하지 않는다**.

### 기존 저장 rlog에서 버스별 CAN 건수만 내보내는 방법

원본 rlog가 설치된 오픈파일럿 개발 환경에서만:

~~~bash
python opendbc_repo/opendbc/car/hyundai/ev6_spas_rlog_extract.py \
  --input '/path/to/previously_saved/rlog.zst' \
  --output '/path/to/private_filtered_ev6_can.csv' \
  --summary-output '/path/to/shareable_bus_counts.json'
~~~

- 로컬 CSV는 각 메시지의 원시 CAN 데이터를 포함하므로 개인적으로 보관한다.
- 별도 JSON 파일은 **CAN ID 및 버스별 수신 건수만** 기록한다.
  CAN payload·개별 타임스탬프·GPS·영상·sendcan은 요약 JSON에 없다.
  이 JSON으로도 ECU 출처, 진단 송신 여부 또는 안전 상태는 확정하지 못한다.
- CSV 생성 중 실패하면 불완전한 임시파일을 지우고 기존 출력 파일을
  보존하도록 **원자적 교체** 방식으로 보강했다.
- 장치에 시험용 제어기·송신 코드 설치/활성화는 필요하지 않다.
  파일 추출 과정은 원본 저장 로그를 읽는 것 외에 어떤 CAN 송신도 하지 않는다.

이번 코드에는 기존 Python 96개 + 버스 출처 회귀 4개 + 원자적 추출/
요약 JSON 4개 = **104/104 로컬 테스트 통과**, syntax 검사 통과.
GitHub CI와 전체 panda C integration의 성공 기록은 여전히 없다.
원본 전체 rlog 파일은 현재 접근 가능한 파일 자료에서 확인되지
않았으므로 **사용자 EV6의 실 물리 RX 버스가 무엇인지 여전히 미확인**.

**실차 적용 NO-GO / Draft PR #47 유지.**
