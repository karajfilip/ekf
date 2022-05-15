# ARIAC 2022 - FER
ARIAC is a competition organized by the National Institute of Standards and Technology (NIST). A focus on the competition is to test the agility of industrial robot system, with the goal of enabling industrial robots to be more productive and more autonomous.

## About us
FER is Faculty of Electrical Engineering and Computing in Zagreb, Croatia. Our team consists of five members.

|       Name        |          E-mail          |       Git        |
| :---------------: | :----------------------: | :--------------: |
|Denis Đurašinović  | denis.durasinovic@fer.hr | De4d56           |
|Filip Karaj        | filip.karaj@fer.hr       | karajfilip       |
|Josip Ante Kozulić | josip-ante.kozulic@fer.hr| josipantekozulic |
|Marija Piliškić    | marija.piliskic@fer.hr   | marijapili       |
|Filip Pušnik       | filip.pusnik@fer.hr      | filippusnik      |

## Installation

In your workspace, clone the [ARIAC](https://github.com/usnistgov/ARIAC):

```bash
git clone https://github.com/usnistgov/ARIAC.git
```
After that clone this repo:

```bash
git clone https://github.com/karajfilip/ariac22.git
```

In your workspace run `catkin build`:
```bash
cd ~/<developer_workspace>
catkin build
```

NOTE: The system is developed for ROS Melodic. If you use other ROS distro, you can use the Docker container that is set up by Docker image specifed in `/ariac22/ariac21`.

## Run

```bash
source <developer_worspace>/devel/setup.bash
roslaunch nist_gear sample_environment.launch load_moveit:=true
rosrun project main_sm.py
```
