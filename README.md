# 동적 장애물 모듈화 코드

## Ubuntu 24.04 설정
---

## 1. Ubuntu 버전 및 ROS 설치 여부 확인

```bash
ls /opt/ros
lsb_release -a
```

---

## 2. Universe 저장소 활성화

ROS 2 설치에 필요한 여러 패키지가 Ubuntu의 `universe` 저장소에 포함되어 있으므로 먼저 활성화함.

```bash
sudo apt update
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
```

---

## 3. Ubuntu 24.04에서 ROS 2 Jazzy 설치 절차

```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

sudo apt update
sudo apt install curl -y

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt upgrade -y
sudo apt install ros-jazzy-desktop -y
```

---

## 4. 설치 후 환경 활성화

설치가 끝나면 아래 명령으로 ROS 2 Jazzy 환경을 활성화함.

```bash
source /opt/ros/jazzy/setup.bash
echo $ROS_DISTRO
```

정상이라면 `jazzy`가 출력되어야 함.

---

## 5. 새 터미널마다 자동 적용

매번 직접 source 하지 않으려면 아래를 실행함.

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 6. 워크스페이스 빌드

프로젝트 디렉터리에서 다음처럼 진행하면 됨.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/dynamic_obstacle_modularization-main
colcon build --symlink-install
source install/setup.bash
```

---

## 7. 추가 권장 패키지

빌드와 의존성 설치를 위해 아래 패키지도 함께 설치하는 것이 좋음.

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep -y
sudo rosdep init
pip install opencv-python ultralytics
rosdep update
```

