# 色付き点群から複数円柱を抽出するRTコンポーネント

大阪電気通信大学  
澤崎 悠太，升谷 保博  
2019年11月24日

## はじめに

- RTC:PCLのPointCloud型（`PointCloudTypes::PointCoud`）で表現された色付きの点群の情報が入力されると，平面の上に垂直に置かれた複数の円柱物体を抽出しその結果を出力するRTコンポーネントです．
- 以下の環境で開発，動作確認しています．
  - Windows 10 64bit版
  - Visual Studio 2015
  - OpenRTM-aist 1.2.0 64bit版
  - Point Cloud Library 1.8.1 AllinOne (VS2015 64bit用)
  - OpenCV 3.4（OpenRTM-aist 1.2.0と一緒にインストールされるもの）

## 処理の概要

- ポート`pc`から入力された色付きの点群のデータを`pcl::PointXYZRGB`に変換する．
- PCLのPassThroughフィルタによって，座標値が`filterXMin` < x < `filterXMax` かつ`filterYMin` < y < `filterYMax` かつ`filterZMin` < z < `filterZMax` 以外の点を削除する．
- PCLのVoxelGridによって，リーフサイズ`leafSize`で点を間引く．
- PCLのNormalEstimationOMPによって，法線ベクトルを求める．
- PCLのSACSegmentationFromNormalsによって，平面を推定し，平面を成す点を削除する．
- 円柱の推定の繰り返し
  - PCLのSACSegmentationFromNormalsによって，最初に推定した平面の法線方向と軸の方向が一致しており，`segmentationRadiusMin` < 半径 < `segmentationRadiusMax`である円柱を推定し，円柱面を成す点を抽出する．
  - 抽出された点の数が`cylinderPointSizeMin`よりも小さければ，繰り返しを抜ける．
  - 抽出された点の色をHSV色空間に変換し，`saturationMin` < 彩度 < `saturationMax` かつ `valueMin` < 明度 < `valueMax`の点だけで色相を`histogramBinNumber`個の区間に分割して頻度分布を求めて，最頻区間の値をこの円柱の色相とする．
  - 点群の重心位置から円柱の軸に垂線を下ろした点を円柱の中心とする．
- 抽出された複数の円柱の内，`cylinderAccumulationMin`回以上ほぼ同じデータを示したものだけを選び，それらの平均値を求め，基準座標系に変換して，まとめてポート`cylinder`へ出力する．

## 仕様

### 入力ポート
- pc
  - 型: PointCloudTypes::PointCloud
  - 概要： 色付きの点群

### 出力ポート
- cylinder
  - 型: RTC::TimedDoubleSeq
  - 概要： 抽出された複数の円柱の情報．円柱1個当たり5要素（中心のx,y,z座標[m]，半径[m]，色相（0～255））

### コンフィギュレーション

- filterXMin
  - 型: float
  - 概要： フィルタのxの最小値 [m]
- filterXMin
  - 型： float
  - デフォルト値： -0.5
  - 概要： フィルタのxの最小値 [m]
- filterXMax
  - 型： float
  - デフォルト値： +0.5
  - 概要： フィルタのxの最大値 [m]
- filterYMin
  - 型： float
  - デフォルト値： -1.0
  - 概要： フィルタのyの最小値 [m]
- filterYMax
  - 型： float
  - デフォルト値： +1.0
  - 概要： フィルタのyの最大値 [m]
- filterZMin
  - 型： float
  - デフォルト値： -1.0
  - 概要： フィルタのzの最小値 [m]
- filterZMax
  - 型： float
  - デフォルト値： -0.4
  - 概要： フィルタのzの最大値 [m]
- leafSize
  - 型： float
  - デフォルト値： 0.01
  - 概要： 間引きのLeafSize [m]
- findingIterationLimit
  - 型： int
  - デフォルト値： 10
  - 概要： 円柱推定の繰り返しの上限値
- segmentationMaxIteration
  - 型： int
  - デフォルト値： 1000
  - 概要： 円柱のセグメンテーションの繰り返しの最大値
- segmentationDistanceThreshold
  - 型： float
  - デフォルト値： 0.05
  - 概要： 円柱のセグメンテーションの距離のしきい値 [m]
- segmentationRadiusMin
  - 型： float
  - デフォルト値： 0.01
  - 概要： 円柱のセグメンテーションの半径の最小値 [m]
- segmentationRadiusMax
  - 型： float
  - デフォルト値： 0.05
  - 概要： 円柱のセグメンテーションの半径の最大値 [m]
- cylinderPointSizeMin
  - 型： int
  - デフォルト値： 100
  - 概要： 円柱と判断するポイント数の最小値
- displayHistogram
  - 型： int
  - デフォルト値： 1
  - 概要： ヒストグラムを表示するかどうか
- histogramBinNumber
  - 型： int
  - デフォルト値： 16
  - 概要： 色相を決める際の色相の区間の数
- saturationMin
  - 型： int
  - デフォルト値： 127
  - 概要： 色相を決める際の彩度の最小値（0～255）
- saturationMax
  - 型： int
  - デフォルト値： 255
  - 概要： 色相を決める際の彩度の最大値（0～255）
- valueMin
  - 型： int
  - デフォルト値： 0
  - 概要： 色相を決める際の明度の最小値（0～255）
- valueMax
  - 型： int
  - デフォルト値： 255
  - 概要： 色相を決める際の明度の最大値（0～255）
- sameCylinderCenterDistanceLimit
  - 型： float
  - デフォルト値： 0.02
  - 概要： 同じ円柱と判断する中心間距離の上限 [m]
- sameCylinderRadiusDistanceLimit
  - 型： float
  - デフォルト値： 0.01
  - 概要： 同じ円柱と判断する半径差の上限 [m]
- sameCylinderHueDistanceLimit
  - 型： float
  - デフォルト値： 23
  - 概要： 同じ円柱と判断する色相差の上限 [deg]
- cylinderAccumulationMin
  - 型： int
  - デフォルト値： 10
  - 概要： 円柱として出力する蓄積回数の最小値
- coordinateTransformationFile
  - 型： string
  - デフォルト値： coordinateTransformation.txt
  - 概要： 座標変換のデータファイル
- calibrationOffsetX
  - 型： float
  - デフォルト値： 0.0
  - 概要： 校正ツールのオフセットx座標 [m]
- calibrationOffsetY
  - 型： float
  - デフォルト値： 0.0
  - 概要： 校正ツールのオフセットy座標 [m]
- calibrationOffsetZ
  - 型： float
  - デフォルト値： 0.0
  - 概要： 校正ツールのオフセットz座標 [m]

## インストール

- [OpenRTM-aist 1.2.0](https://www.openrtm.org/openrtm/ja/download/openrtm-aist-cpp/openrtm-aist-cpp_1_2_0_release)をインストール．
- [GitHubのpclのRelease](https://github.com/PointCloudLibrary/pcl/releases)の中のWindows用AllInOne `PCL-X.X.X-AllInOne-msvcYYYY-winZZ.exe`をインストール．
- [PCToColorCylinder](https://github.com/MasutaniLab/PCToColorCylinder)をクローンかダウンロードする．
- CMake
  - ビルドディレクトリはトップ直下の`build`
  - ConfigureはVisual Studio 64bit
- `build\PCToColorCylinder.sln`をVisual Studioで開く．
- ビルド

## 使い方

- 扱うデータ量が多いので，CORBAのデフォルトの設定ではエラーになる．rtc.confに`corba.args: -ORBgiopMaxMsgSize`の設定が必要．トップディレクトリのrtc.confでは`corba.args: -ORBgiopMaxMsgSize 20971520`にしている（デフォルト値の10倍）．
- RTC:PCLのPointCloud型（`PointCloudTypes::PointCoud`）のポートを持つセンサのコンポーネントの出力を入力ポート`pc`に接続する．
- 基準となる座標系（ロボットアームの台座の座標系）などに対するセンサ座標系の姿勢（回転行列）と位置（並進ベクトル）をコンフィギュレーション`coordinateTransformationFile`に指定したファイルに設定する．基準座標系と校正ツールにオフセットがある場合は，コンフィギュレーションの`calibrationOffsetX`, `calibrationOffsetY`,`calibrationOffsetZ`（単位[m]）設定する．
- トップディレクトリに置いているバッチファイル`PCToColorCylinder.bat`を実行する．
- 平面上に置かれた複数の円柱物体をセンサで捉えると，それぞれの位置と半径と色情報（色相）がポート`cylinder`に出力される．

## 既知の問題・TODO

- センサの検出範囲内に平面と円柱以外の物体が多数ある場合は，円柱を正しく見つけられない場合があります．
- 平面に対して軸が垂直な円柱にしか対応していません．
