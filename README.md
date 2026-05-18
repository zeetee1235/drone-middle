# Drone Middle

한국로봇항공기경연대회 중급부문을 위한 실내 GPS 비의존 자율비행 드론 프로젝트입니다.

## 개요

이 프로젝트는 바닥 격자선과 ArUco 마커를 이용해 드론이 실내에서 위치를 추정하고, 마커 탐색, 구조 경로 계획, 귀환 및 착륙까지 수행하는 것을 목표로 합니다.

주요 특징은 다음과 같습니다.

- ROS2 C++ 기반 인식, 항법, 미션 관리, PX4 Offboard 제어 노드
- Gazebo/PX4 SITL 및 headless 시뮬레이션 스크립트
- 교차점 기반 위치 보정과 고속 스프린트 경로 계획
- 기술 보고서, 실험 결과 그림, mission-cost sweep 자료 포함

## 폴더 구조

- `src/`: ROS2 패키지
- `scripts/`: 실행, 시뮬레이션, 검증 스크립트
- `reports/`: 기술개발계획서와 실험 그림
- `docs/`: 설계 문서와 규정 정리
- `tools/`: 분석 및 그림 생성 도구

## 참고

`build/`, `install/`, `log/`, `bags/`, `external/`은 로컬 빌드/실험 산출물이므로 Git 추적에서 제외했습니다.

`src/px4_msgs`는 PX4 메시지 패키지 submodule입니다.
