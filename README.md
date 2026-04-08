# 동적 장애물 모듈화 코드

## Ubuntu 24.04 설정

현재 환경 기준 정리:

- OS: Ubuntu 24.04.4 LTS (noble)
- 상태: ROS 2 미설치 상태에서 시작
- 확인 결과:
  - `/opt/ros` 디렉터리가 없었음
  - 따라서 `source /opt/ros/jazzy/setup.bash`가 동작하지 않았음
- 결론:
  - Ubuntu 24.04에서는 ROS 2 **Jazzy**를 사용하는 것이 맞음
  - 다만 당시에는 Jazzy가 아직 설치되지 않아 먼저 설치가 필요했음

---

## 1. Ubuntu 버전 및 ROS 설치 여부 확인

```bash
ls /opt/ros
lsb_release -a
```

확인 결과:

- `ls /opt/ros` → 디렉터리 없음
- `lsb_release -a` → Ubuntu 24.04.4 LTS

즉, ROS 2가 아직 설치되지 않은 상태였음.

---

## 2. Universe 저장소 활성화

ROS 2 설치에 필요한 여러 패키지가 Ubuntu의 `universe` 저장소에 포함되어 있으므로 먼저 활성화함.

```bash
sudo apt update
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
```

---

## 3. ROS 2 저장소 설정 중 확인한 사항

처음에는 아래 명령을 시도했음.

```bash
sudo apt update
sudo apt install curl -y
sudo apt install ros-apt-source -y
```

이때 다음 에러가 발생했음.

```bash
E: ros-apt-source 패키지를 찾을 수 없습니다
```

원인:

- 패키지명이 `ros-apt-source`가 아니라 **`ros2-apt-source`**가 맞음

---

## 4. Ubuntu 24.04에서 ROS 2 Jazzy 설치 절차

다음 순서로 진행하면 됨.

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

## 5. 설치 후 환경 활성화

설치가 끝나면 아래 명령으로 ROS 2 Jazzy 환경을 활성화함.

```bash
source /opt/ros/jazzy/setup.bash
echo $ROS_DISTRO
```

정상이라면 `jazzy`가 출력되어야 함.

---

## 6. 새 터미널마다 자동 적용

매번 직접 source 하지 않으려면 아래를 실행함.

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 7. 워크스페이스 빌드

프로젝트 디렉터리에서 다음처럼 진행하면 됨.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/dynamic_obstacle_modularization-main
colcon build --symlink-install
source install/setup.bash
```

---

## 8. 추가 권장 패키지

빌드와 의존성 설치를 위해 아래 패키지도 함께 설치하는 것이 좋음.

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep -y
sudo rosdep init
rosdep update
```

필요하면 이후에는 다음처럼 의존성을 먼저 설치할 수 있음.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/dynamic_obstacle_modularization-main
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## 9. 핵심 정리

- Ubuntu 24.04에서는 보통 **ROS 2 Jazzy**를 사용함
- `/opt/ros/jazzy/setup.bash`가 없었던 이유는 **Jazzy가 설치되지 않았기 때문**
- `universe` 저장소를 활성화한 뒤 ROS 2 저장소를 추가하고 Jazzy를 설치해야 함
- `ros-apt-source`가 아니라 **`ros2-apt-source`**가 맞는 패키지명임
