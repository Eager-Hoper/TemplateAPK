# Overview
Eager Hoper's repository for Kibo-RPC 2023

<img width="1105" alt="スクリーンショット 2023-08-20 12 35 23" src="https://github.com/Eager-Hoper/TemplateAPK/assets/61105696/c6f9ab9f-38c0-4a04-8a36-dfa763b40ead">

Cited from [here](https://jaxa.krpc.jp/).

# Guidebook and Rulebook
The guidebook and rulebook for this tournament are available [here](https://jaxa.krpc.jp/download)!

# Main code
[YourService.java](app/src/main/java/jp/jaxa/iss/kibo/rpc/japan/YourService.java)


# Code Description
## function
| Function         | Description |
|------------------|-------------|
| moveAndShot      | 移動、レーザーオン、スナップショットを撮影            |
| MoveTo           | 移動。api.moveToをオーバーライド            |
| image_correction | NavCamで取得された画像の歪みを補正            |
| ReadQR           | QRコードからメッセージを読み取る            |
| checkMissionTime | ミッションの残り時間を取得（*分/5分）            |
| checkActiveTime  | 現在アクティブなターゲットマーカーの残り時間を取得（*分/2分）            |
| getRelative      | 現在位置と、 ターゲットマーカー中心の相対距離を取得           |
| AR_detect        | ARマーカ検出            |
| getTargetCenter  | ターゲットマーカーの中心座標を取得            |
| getScale         | 画像(pixel)と実寸(m)のスケールを取得            |
| reMove_AR_moveTo | ARマーカを用いて算出したターゲットマーカー中心に移動            |
| getMatNavCam     | NavCamで画像を取得            |

## About const values
### times[][]
`times` stores the time it takes to travel between each point.
Use this [MakeTimesArray_for_final.ipynb](https://drive.google.com/drive/folders/1_0-XkhO3x_m994M7ZXwrtSrgdycwxsO7?usp=sharing) python script to get the travel time for each route from the log. Then, if the travel time is greater than the travel time in the previous simulation, change the value of timesArray.

## API
The following api was used. For more information about api, please refer to the [programming manual](https://jaxa.krpc.jp/download).

- getRobotKinematics
- getMatNavCam
- flashlightControlFront
- moveTo
- laserControl
- startMission
- notifyGoingToGoal
- reportMissionCompletion
- takeTargetSnapshot
- getNavCamIntrinsics
- saveMatImage
- getTimeRemaining
- getActiveTargets

## Library
- OpenCV
  - Aruco: AR marker
  - QR
 
## Algorithm for travel paths
Coming soon...

## Flowchart of this code
[Flowchart（Miro）](https://miro.com/welcomeonboard/REYxb1NUS0tRR0hBZTBubXFTUUNWU1JSMU43SzJZSWR6Q3l0VWdVMHRxWTVoanhFTGhOSVJqY3VBcjZCWjViZnwzNDU4NzY0NTMxMjAyMzk0NTI2fDI=?share_link_id=431351260785)


# Appendix
-------------------------------
## About Astrobee

![スクリーンショット 2023-08-20 12 49 22](https://github.com/Eager-Hoper/TemplateAPK/assets/61105696/f8219a4e-108e-428b-98db-97af5492c271)

![スクリーンショット 2023-08-20 12 49 47](https://github.com/Eager-Hoper/TemplateAPK/assets/61105696/4730e4f7-5242-4ded-b721-cafb63d9f2e7)

Cited from [here](https://jaxa.krpc.jp/).

## Command

```
git add .
git commit -m "add: コミットメッセージ"
git push origin main

git pull origin main (or your_name)
```

## TIPS
### LOGに値を出力するとき
[LOGに値を出力する](https://github.com/Eager-Hoper/TemplateAPK/blob/594ceeeb68bf1dd1ec7e12e4fa5ceb919d50aef4/app/src/main/java/jp/jaxa/iss/kibo/rpc/testapk/YourService.java#L45)

