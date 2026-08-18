# tetra_config

TETRA 로봇의 구동, 매핑, 내비게이션을 위한 각종 설정과 런치(launch) 파일들을 모아둔 패키지 그룹입니다.

## 패키지 구성
- `tetra_bringup`: 로봇 드라이버, 센서 노드 및 기본 구동을 시작하기 위한 통합 런치 파일 패키지
- `tetra_cartographer`: 2D/3D SLAM을 위한 Google Cartographer 설정 및 런치 패키지
- `tetra_navigation2`: Nav2(Navigation2)를 이용한 자율 주행 및 경로 계획 설정 패키지
