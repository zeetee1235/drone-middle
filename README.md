# Drone Middle

한국로봇항공기경연대회 중급부문을 위한 실내 GPS 비의존 자율비행 드론 프로젝트입니다.

## 폴더 구조

- `src/`: ROS2 패키지
- `scripts/`: 실행, 시뮬레이션, 검증 스크립트
- `reports/`: 기술개발계획서와 실험 그림
- `docs/`: 설계 문서와 규정 정리
- `tools/`: 분석 및 그림 생성 도구

## 참고

`build/`, `install/`, `log/`, `bags/`, `external/`은 로컬 빌드/실험 산출물이므로 Git 추적에서 제외

`src/px4_msgs`는 PX4 메시지 패키지 submodule입니다.
