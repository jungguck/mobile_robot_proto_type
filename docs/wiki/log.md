# 📜 Wiki Log

이 파일은 위키에 가해진 모든 변경 사항을 연대순으로 기록합니다.

## [2026-05-20] Initialize | LLM Wiki 패턴 활성화
- **내용**: `firs_you_need_it.md` 기반의 지식 관리 체계 구축.
- **작업**: 
    - `docs/wiki/` 구조 생성.
    - `SCHEMA.md`, `index.md` 초기화.
    - README rewrite 내용을 기반으로 첫 번째 지식 통합(Ingest) 준비.

## [2026-05-20] Ingest | 초기 프로젝트 지식 통합
- **내용**: README 및 코드 분석을 통한 핵심 지식 위키화.
- **작업**:
    - [[System_Architecture]] 페이지 생성: 레이어 구조 및 데이터 흐름 정의.
    - [[Relay_Robot_Hardware]] 페이지 생성: 모터, 센서 사양 기록.
    - [[Debugging_Experience]] 페이지 생성: TF 충돌, 좀비 프로세스 등 해결 사례 정리.

## [2026-06-14] Bugfix | MPC+SLAM 파이프라인 코드 분석 및 버그 수정
- **내용**: Plan/평가 에이전트 기반 전체 파이프라인 분석 후 코드 버그 2건 수정.
- **작업**:
    - [[Debugging_Experience#5]] RPM 변환 계수 `/6.0` → `/600.0` 수정 (`real_robot_driver_260519.py`)
    - [[Debugging_Experience#6]] 왼쪽 모터 피드백 부호 반전 누락 수정 (`rpm_L = -rpm_L` 추가)
    - `my_slam_params.yaml` `use_sim_time: true` → `false` 수정 (dead config 혼란 방지)
    - `README_ROBOT.md` 생성: 파일 관계도, 토픽/TF 흐름, 6단계 실행 가이드
- **미결**: RPLidar 모델별 scan_mode 실측 확인, EKF 공분산 튜닝
