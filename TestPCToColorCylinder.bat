@echo off
:PCToColorCylinderのテスト

:ネーミングサービスの確認
rtls /localhost > nul
if errorlevel 1 (
  echo ネーミングサーバが見つかりません
  pause
  exit /b 1
  rem /bオプションは親を終わらせないために必須
)


:コンポーネントの起動
call ..\Kinect2ToPC\Kinect2ToPC.bat
call ..\PCToColorCylinder\PCToColorCylinder.bat

:コンポーネント名を変数化
set k=/localhost/Kinect2ToPC0.rtc
set c=/localhost/PCToColorCylinder0.rtc

:時間待ち
timeout 5

:接続
rtcon %k%:pc %c%:pc

:アクティベート
rtact %k% %c%

:loop
set /p ans="終了しますか？ (y/n)"
if not "%ans%"=="y" goto loop

:ディアクティベート
rtdeact %k% %c%

:終了（rtexitは，引数を一つずつ）
for %%i in (%k% %c%) do (
  rtexit %%i
)
