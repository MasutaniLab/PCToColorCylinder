// -*- C++ -*-
/*!
 * @file  PCToColorCylinder.cpp
 * @brief ModuleDescription
 * @date $Date$
 *
 * $Id$
 */

#include "PCToColorCylinder.h"

#include <iostream>
#include <iomanip>
#include <cmath>
using namespace std;

#define print(x) cout << #x << ": " << x << endl
#define printnl(x) cout << #x << ": " << endl << x << endl
#define printPointField(x) cout << #x << ": " << x.name << "," << x.offset << "," << x.data_type << "," << x.count << endl

typedef pcl::PointXYZRGB PointType;

void setPointCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
  pcl::PointCloud<PointType>::Ptr cloud, string name,
  bool colored = false, int r = 255, int g = 255, int b = 255);
void setCube(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
  const Eigen::Vector3f &center, const Eigen::Quaternionf &rotation, float size,
  string name, int r, int g, int b);
void reviseCylinder(pcl::PointCloud<PointType>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients,
  Eigen::Vector3f &center, Eigen::Quaternionf &rotation, float &radius);
float calcHue(pcl::PointCloud<PointType>::ConstPtr cloud, int hbins, int smin, int smax, int vmin, int vmax, string title = "");
void hsv2rgb(int h, int s, int v, int &r, int &g, int &b);
bool checkRotationMatrix(const Eigen::Affine3f &a);

// Module specification
// <rtc-template block="module_spec">
static const char* pctocolorcylinder_spec[] =
  {
    "implementation_id", "PCToColorCylinder",
    "type_name",         "PCToColorCylinder",
    "description",       "Extract cylinders from point cloud with color",
    "version",           "1.0.0",
    "vendor",            "MasutaniLab",
    "category",          "PointCloud",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.filterXMin", "-0.5",
    "conf.default.filterXMax", "+0.5",
    "conf.default.filterYMin", "-1.0",
    "conf.default.filterYMax", "+1.0",
    "conf.default.filterZMin", "-1.0",
    "conf.default.filterZMax", "-0.4",
    "conf.default.leafSize", "0.01",
    "conf.default.findingIterationLimit", "10",
    "conf.default.segmentationMaxIteration", "1000",
    "conf.default.segmentationDistanceThreshold", "0.05",
    "conf.default.segmentationRadiusMin", "0.01",
    "conf.default.segmentationRadiusMax", "0.05",
    "conf.default.cylinderPointSizeMin", "100",
    "conf.default.displayHistogram", "1",
    "conf.default.histogramBinNumber", "16",
    "conf.default.saturationMin", "127",
    "conf.default.saturationMax", "255",
    "conf.default.valueMin", "0",
    "conf.default.valueMax", "255",
    "conf.default.sameCylinderCenterDistanceLimit", "0.02",
    "conf.default.sameCylinderRadiusDistanceLimit", "0.01",
    "conf.default.sameCylinderHueDistanceLimit", "23",
    "conf.default.cylinderAccumulationMin", "10",
    "conf.default.coordinateTransformationFile", "coordinateTransformation.txt",
    "conf.default.calibrationOffsetX", "0.0",
    "conf.default.calibrationOffsetY", "0.0",
    "conf.default.calibrationOffsetZ", "0.0",

    // Widget
    "conf.__widget__.filterXMin", "text",
    "conf.__widget__.filterXMax", "text",
    "conf.__widget__.filterYMin", "text",
    "conf.__widget__.filterYMax", "text",
    "conf.__widget__.filterZMin", "text",
    "conf.__widget__.filterZMax", "text",
    "conf.__widget__.leafSize", "text",
    "conf.__widget__.findingIterationLimit", "text",
    "conf.__widget__.segmentationMaxIteration", "text",
    "conf.__widget__.segmentationDistanceThreshold", "text",
    "conf.__widget__.segmentationRadiusMin", "text",
    "conf.__widget__.segmentationRadiusMax", "text",
    "conf.__widget__.cylinderPointSizeMin", "text",
    "conf.__widget__.displayHistogram", "text",
    "conf.__widget__.histogramBinNumber", "text",
    "conf.__widget__.saturationMin", "text",
    "conf.__widget__.saturationMax", "text",
    "conf.__widget__.valueMin", "text",
    "conf.__widget__.valueMax", "text",
    "conf.__widget__.sameCylinderCenterDistanceLimit", "text",
    "conf.__widget__.sameCylinderRadiusDistanceLimit", "text",
    "conf.__widget__.sameCylinderHueDistanceLimit", "text",
    "conf.__widget__.cylinderAccumulationMin", "text",
    "conf.__widget__.coordinateTransformationFile", "text",
    "conf.__widget__.calibrationOffsetX", "text",
    "conf.__widget__.calibrationOffsetY", "text",
    "conf.__widget__.calibrationOffsetZ", "text",
    // Constraints

    "conf.__type__.filterXMin", "float",
    "conf.__type__.filterXMax", "float",
    "conf.__type__.filterYMin", "float",
    "conf.__type__.filterYMax", "float",
    "conf.__type__.filterZMin", "float",
    "conf.__type__.filterZMax", "float",
    "conf.__type__.leafSize", "float",
    "conf.__type__.findingIterationLimit", "int",
    "conf.__type__.segmentationMaxIteration", "int",
    "conf.__type__.segmentationDistanceThreshold", "float",
    "conf.__type__.segmentationRadiusMin", "float",
    "conf.__type__.segmentationRadiusMax", "float",
    "conf.__type__.cylinderPointSizeMin", "int",
    "conf.__type__.displayHistogram", "int",
    "conf.__type__.histogramBinNumber", "int",
    "conf.__type__.saturationMin", "int",
    "conf.__type__.saturationMax", "int",
    "conf.__type__.valueMin", "int",
    "conf.__type__.valueMax", "int",
    "conf.__type__.sameCylinderCenterDistanceLimit", "float",
    "conf.__type__.sameCylinderRadiusDistanceLimit", "float",
    "conf.__type__.sameCylinderHueDistanceLimit", "float",
    "conf.__type__.cylinderAccumulationMin", "int",
    "conf.__type__.coordinateTransformationFile", "string",
    "conf.__type__.calibrationOffsetX", "float",
    "conf.__type__.calibrationOffsetY", "float",
    "conf.__type__.calibrationOffsetZ", "float",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
PCToColorCylinder::PCToColorCylinder(RTC::Manager* manager)
// <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_pcIn("pc", m_pc),
    m_cylinderOut("cylinder", m_cylinder)

  // </rtc-template>
{
}

/*!
 * @brief destructor
 */
PCToColorCylinder::~PCToColorCylinder()
{
}



RTC::ReturnCode_t PCToColorCylinder::onInitialize()
{
  RTC_INFO(("onInitialize()"));
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("pc", m_pcIn);
  
  // Set OutPort buffer
  addOutPort("cylinder", m_cylinderOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("filterXMin", m_filterXMin, "-0.5");
  bindParameter("filterXMax", m_filterXMax, "+0.5");
  bindParameter("filterYMin", m_filterYMin, "-1.0");
  bindParameter("filterYMax", m_filterYMax, "+1.0");
  bindParameter("filterZMin", m_filterZMin, "-1.0");
  bindParameter("filterZMax", m_filterZMax, "-0.4");
  bindParameter("leafSize", m_leafSize, "0.01");
  bindParameter("findingIterationLimit", m_findingIterationLimit, "10");
  bindParameter("segmentationMaxIteration", m_segmentationMaxIteration, "1000");
  bindParameter("segmentationDistanceThreshold", m_segmentationDistanceThreshold, "0.05");
  bindParameter("segmentationRadiusMin", m_segmentationRadiusMin, "0.01");
  bindParameter("segmentationRadiusMax", m_segmentationRadiusMax, "0.05");
  bindParameter("cylinderPointSizeMin", m_cylinderPointSizeMin, "100");
  bindParameter("displayHistogram", m_displayHistogram, "1");
  bindParameter("histogramBinNumber", m_histogramBinNumber, "16");
  bindParameter("saturationMin", m_saturationMin, "127");
  bindParameter("saturationMax", m_saturationMax, "255");
  bindParameter("valueMin", m_valueMin, "0");
  bindParameter("valueMax", m_valueMax, "255");
  bindParameter("sameCylinderCenterDistanceLimit", m_sameCylinderCenterDistanceLimit, "0.02");
  bindParameter("sameCylinderRadiusDistanceLimit", m_sameCylinderRadiusDistanceLimit, "0.01");
  bindParameter("sameCylinderHueDistanceLimit", m_sameCylinderHueDistanceLimit, "23");
  bindParameter("cylinderAccumulationMin", m_cylinderAccumulationMin, "10");
  bindParameter("coordinateTransformationFile", m_coordinateTransformationFile, "coordinateTransformation.txt");
  bindParameter("calibrationOffsetX", m_calibrationOffsetX, "0.0");
  bindParameter("calibrationOffsetY", m_calibrationOffsetY, "0.0");
  bindParameter("calibrationOffsetZ", m_calibrationOffsetZ, "0.0");
  // </rtc-template>

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t PCToColorCylinder::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PCToColorCylinder::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PCToColorCylinder::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t PCToColorCylinder::onActivated(RTC::UniqueId ec_id)
{
  RTC_INFO(("onActivated()"));
  m_rtcError = false;  //OpenRTM-aist-1.2.0のバグ回避

  print(m_coordinateTransformationFile);
  ifstream ifs(m_coordinateTransformationFile.c_str());
  if (!ifs) {
    RTC_ERROR(("%sを開けません", m_coordinateTransformationFile.c_str()));
    m_rtcError = true;
    return RTC::RTC_ERROR;
  }
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      ifs >> m_coordinateTransformation.linear()(i, j);
    }
  }
  for (int i = 0; i < 3; i++) {
    ifs >> m_coordinateTransformation.translation()(i);
  }

  printnl(m_coordinateTransformation.matrix());
  if (!checkRotationMatrix(m_coordinateTransformation)) {
    RTC_ERROR(("回転行列の成分が不適切"));
    m_rtcError = true;
    return RTC::RTC_ERROR;
  }
  Eigen::Affine3f offset;
  offset = Eigen::Translation3f(m_calibrationOffsetX, m_calibrationOffsetY, m_calibrationOffsetZ);
  m_coordinateTransformation = offset * m_coordinateTransformation;
  printnl(m_coordinateTransformation.matrix());
#if 1
  //点群の座標系のX軸とZ軸を逆転させる（推定プログラムとKinect2ToPCで座標系の仕様が異なるため）
  Eigen::Affine3f roty;
  roty = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY());
  m_coordinateTransformation = m_coordinateTransformation * roty;
  printnl(m_coordinateTransformation.matrix());
#endif

  m_viewer.reset(new pcl::visualization::PCLVisualizer("PCToCylinde"));
  m_viewer->setBackgroundColor(0, 0, 0);
  m_viewer->addCoordinateSystem(1.0);
  m_viewer->setCameraPosition(0, 0, 3, 0, 0, 0, 0, 1, 0);
  m_viewer->setCameraClipDistances(0, 10);

  m_first = true;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t PCToColorCylinder::onDeactivated(RTC::UniqueId ec_id)
{
  RTC_INFO(("onDeactivate()"));
  if (m_viewer != nullptr) {
    m_viewer->close();
  }

  return RTC::RTC_OK;
}


RTC::ReturnCode_t PCToColorCylinder::onExecute(RTC::UniqueId ec_id)
{
  if (m_rtcError) return RTC::RTC_ERROR; //OpenRTM-aist-1.2.0のバグ回避

  if (m_pcIn.isNew()) {
    m_pcIn.read();
    if (m_first) {
      print(m_pc.seq);
      print(m_pc.height);
      print(m_pc.width);
      print(m_pc.type);
      print(m_pc.is_bigendian);
      print(m_pc.point_step);
      print(m_pc.row_step);
      print(m_pc.is_dense);
      for (int i = 0; i < m_pc.fields.length(); i++) {
        cout << i << " ";
        printPointField(m_pc.fields[i]);
      }
      m_first = false;
    }
    string type = m_pc.type;
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    if (type == "xyzrgb" || type == "xyzrgba") {
      cloud->is_dense = m_pc.is_dense;
      cloud->points.resize(m_pc.width*m_pc.height);
      float *src = (float *)m_pc.data.get_buffer();
      for (size_t i = 0; i < cloud->points.size(); i++) {
        cloud->points[i].x = src[0];
        cloud->points[i].y = src[1];
        cloud->points[i].z = src[2];
        cloud->points[i].rgb = src[3];
        src += 4;
      }
    } else {
      RTC_ERROR(("type %s not suppoted", type.c_str()));
      return RTC::RTC_ERROR;
    }
    //cout << "cloud->points.size(): " << cloud->points.size() << endl;
    m_viewer->removeAllPointClouds();
    m_viewer->removeAllShapes();

#if 0
    //最初の点群を表示
    setPointCloud(m_viewer, cloud, "cloud");
#endif

    //座標値によるフィルタリング
    pcl::PointCloud<PointType>::Ptr cloud_passed(new pcl::PointCloud<PointType>);

    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(m_filterXMin, m_filterXMax);
    pass.filter(*cloud_passed);

    pass.setInputCloud(cloud_passed);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(m_filterYMin, m_filterYMax);
    pass.filter(*cloud_passed);

    pass.setInputCloud(cloud_passed);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(m_filterZMin, m_filterZMax);
    pass.filter(*cloud_passed);
    //cout << "cloud_passed->points.size(): " << cloud_passed->points.size() << endl;

#if 0
    //フィルタリング後の点群を表示
    setPointCloud(m_viewer, cloud_passed, "cloud_passed");
#endif

    //間引き
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(cloud_passed);
    sor.setLeafSize(m_leafSize, m_leafSize, m_leafSize);
    sor.setDownsampleAllData(true);
    pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);
    sor.filter(*cloud_filtered);
    //cout << "cloud_filtered->points.size(): " << cloud_filtered->points.size() << endl;

#if 1
    //間引き後の点群を表示
    setPointCloud(m_viewer, cloud_filtered, "cloud_filtered");
#endif

    //点が存在しないと法線ベクトル群の算出で破綻するので終了
    if (cloud_filtered->points.size() == 0) {
      RTC_INFO(("cloud_filtered->points.size() == 0"));
      return RTC::RTC_OK;
    }

    //法線ベクトル群の算出
    pcl::NormalEstimationOMP<PointType, pcl::Normal> ne;
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setRadiusSearch(0.03);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*cloud_normals);
    //cout << "cloud_normals->points.size(): " << cloud_normals->points.size() << endl;

#if 0
    //法線ベクトル群の表示
    m_viewer->addPointCloudNormals<PointType, pcl::Normal>(cloud_filtered, cloud_normals, 10, 0.05, "cloud_normals");
#endif

    //平面の推定
    pcl::SACSegmentationFromNormals<PointType, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(50);
    seg.setDistanceThreshold(0.03);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    seg.segment(*inliers_plane, *coefficients_plane);
    //cout << "*coefficients_plane: " << *coefficients_plane << endl;

    //平面の抽出
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_plane);
    extract.setNegative(false);
    pcl::PointCloud<PointType>::Ptr cloud_plane(new pcl::PointCloud<PointType>());
    extract.filter(*cloud_plane);
    //cout << "cloud_plane->points.size(): " << cloud_plane->points.size() << endl;

#if 0
    //平面の点群の表示
    setPointCloud(m_viewer, cloud_plane, "cloud_plane", true, 255, 255, 0);
#endif

    //平面以外の点群の抽出
    extract.setNegative(true);
    pcl::PointCloud<PointType>::Ptr cloud_filtered2(new pcl::PointCloud<PointType>());
    extract.filter(*cloud_filtered2);

#if 1
    //平面以外の点群の表示
    setPointCloud(m_viewer, cloud_filtered2, "cloud_filtered2", true, 0, 255, 255);
#endif

    //平面以外の法線ベクトル群の抽出
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setNegative(true);
    extract_normals.setIndices(inliers_plane);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
    extract_normals.filter(*cloud_normals2);

    m_cylinder.data.length(0);
    bool found = false;

    vector<Cylinder> cylinders;

    //円柱の推定を繰り返す
    for (int i = 0; i < m_findingIterationLimit; i++) {
      //cout << "-----------------------------------" << endl;
      //cout << "i: " << i << endl;
      //円柱の推定
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_CYLINDER);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setNormalDistanceWeight(0.1);
      seg.setMaxIterations(m_segmentationMaxIteration);
      seg.setDistanceThreshold(m_segmentationDistanceThreshold);
      seg.setRadiusLimits(m_segmentationRadiusMin, m_segmentationRadiusMax);
      Eigen::Vector3f axis;
      axis << coefficients_plane->values[0], coefficients_plane->values[1], coefficients_plane->values[2];
      seg.setAxis(axis);
      seg.setEpsAngle(0.1); //rad単位？
      seg.setInputCloud(cloud_filtered2);
      seg.setInputNormals(cloud_normals2);
      pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
      seg.segment(*inliers_cylinder, *coefficients_cylinder);
      //cout << "*coefficients_cylinder: " << *coefficients_cylinder << endl;

      //円柱の抽出
      extract.setInputCloud(cloud_filtered2);
      extract.setIndices(inliers_cylinder);
      extract.setNegative(false);
      pcl::PointCloud<PointType>::Ptr cloud_cylinder(new pcl::PointCloud<PointType>());
      extract.filter(*cloud_cylinder);
      //cout << "cloud_cylinder->points.size(): " << cloud_cylinder->points.size() << endl;

      //点の数が少なければ処理を終了する．
      if (cloud_cylinder->points.size() < m_cylinderPointSizeMin) break;

      found = true;

      //displayHistogramが1ならば最初の円柱だけヒストグラムを表示
      string name;
      if (m_displayHistogram == 1 && i == 0) {
        name = "histogram";
      }
      float hue = calcHue(cloud_cylinder, m_histogramBinNumber, 
        m_saturationMin, m_saturationMax, m_valueMin, m_valueMax, name);
      //cout << "hue: " << hue << endl;

      int r, g, b;
      if (hue < 0) {
        r = g = b = 255; //色相が特定できなかった場合は白色
      } else {
        hsv2rgb(hue / 2, 255, 255, r, g, b);
      }

      string si = to_string(i);

#if 1
      //円柱の点群の表示
      setPointCloud(m_viewer, cloud_cylinder, "cloud_cylinder" + si, true, r, g, b);
#endif

      //円柱の位置補正
      Eigen::Vector3f center;
      Eigen::Quaternionf rotation;
      float radius;
      reviseCylinder(cloud_cylinder, coefficients_cylinder, center, rotation, radius);

      //円柱の位置を表す立方体を表示
#if 1
      setCube(m_viewer, center, rotation, radius * 2, "cube" + si, r, g, b);
#endif
      //円柱以外の点群を抽出
      extract.setNegative(true);
      extract.filter(*cloud_filtered2);

      //円柱以外の法線ベクトル群を抽出
      extract_normals.setInputCloud(cloud_normals2);
      extract_normals.setNegative(true);
      extract_normals.setIndices(inliers_cylinder);
      extract_normals.filter(*cloud_normals2);

      //結果
      Cylinder c(center, radius, hue);
      cylinders.push_back(c);
      RTC_INFO(("before i: %d, c: (%6.3f, %6.3f, %6.3f), r: %6.3f, h: %3.0f", 
        i, center.x(), center.y(), center.z(), radius, hue));
    }
    //cout << "===================================" << endl;
    if (!found) {
      RTC_INFO(("not found"));
    }
    selectCylinders(cylinders, m_cylinder);
    for (size_t i = 0; i < m_cylinder.data.length() / 5; i++) {
      RTC_INFO(("after  i: %d, c: (%6.3f, %6.3f, %6.3f), r: %6.3f, h: %3.0f",
        i, m_cylinder.data[5 * i + 0], m_cylinder.data[5 * i + 1], m_cylinder.data[5 * i + 2], 
        m_cylinder.data[5 * i + 3], m_cylinder.data[5 * i + 4]));
    }
    m_cylinderOut.write();
  }

  m_viewer->spinOnce();
  if (m_viewer->wasStopped()) {
    // Deactivate self
  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t PCToColorCylinder::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PCToColorCylinder::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PCToColorCylinder::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PCToColorCylinder::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PCToColorCylinder::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

//機能：点群から得られた円柱情報を蓄え、妥当なものだけ選択し、平均し、座標変換し、TimedDoubleSeqの形式にする。
//引数： cylinders 円柱情報（入力）、seq 選択後の円柱情報（出力）
//戻り値： なし
void PCToColorCylinder::selectCylinders(const std::vector<Cylinder> &cylinders, RTC::TimedDoubleSeq &seq)
{
#if 1
  //全ての候補の更新をfalseに
  list<CylinderCandidate>::iterator j;
  for (j = m_cylinderCandidates.begin(); j != m_cylinderCandidates.end(); ++j) {
    j->updated = false;
  }
  //あらたな円柱情報を順番に処理
  vector<Cylinder>::const_iterator i;
  for (i = cylinders.begin(); i != cylinders.end(); ++i) {
    //候補の中から1番近い円柱を求める
    float dmin = FLT_MAX;
    list<CylinderCandidate>::iterator jmin;
    for (j = m_cylinderCandidates.begin(); j != m_cylinderCandidates.end(); ++j) {
      float d = (i->center - j->cylinder.center).norm();
      if (d < dmin) {
        dmin = d;
        jmin = j;
      }
    }
#if  0
    if (dmin != FLT_MAX) {
      print(jmin->cylinder.center);
      print(dmin);
      print(abs(i->radius - jmin->cylinder.radius));
      print(abs(i->hue - jmin->cylinder.hue));
    }
#endif
    //1番近い候補の円柱との距離が小さく、かつ、半径が近く、かつ、色相が近ければ、1番近い候補を更新する
    if (dmin != FLT_MAX 
      && dmin < m_sameCylinderCenterDistanceLimit
      && abs(i->radius - jmin->cylinder.radius) < m_sameCylinderRadiusDistanceLimit
      && abs(i->hue - jmin->cylinder.hue) < m_sameCylinderHueDistanceLimit) {
      //蓄積されたものと平均する
      jmin->cylinder.center = ((jmin->cylinder.center)*(jmin->count) + i->center) / (jmin->count + 1);
      jmin->cylinder.radius = ((jmin->cylinder.radius)*(jmin->count) + i->radius) / (jmin->count + 1);
      jmin->cylinder.hue = ((jmin->cylinder.hue)*(jmin->count) + i->hue) / (jmin->count + 1);
      jmin->count++;
      jmin->updated = true;
    } else {
      //そうでなければ、候補に新たに追加する
      CylinderCandidate c;
      c.cylinder = *i;
      c.count = 1;
      c.updated = true;
      m_cylinderCandidates.push_back(c);
    }
  }
  //更新されなかったものは削除
  for (j = m_cylinderCandidates.begin(); j != m_cylinderCandidates.end();) { //要素削除のためforの更新処理はあえて書いていない
    if (j->updated) {
      ++j;
    } else {
      j = m_cylinderCandidates.erase(j);
    }
  }
  //一定回数以上蓄積した物を座標変換し出力へコピー
  int k = 0;
  seq.data.length(0);
  for (j = m_cylinderCandidates.begin(); j != m_cylinderCandidates.end(); ++j) {
    if (j->count >= m_cylinderAccumulationMin) {
      seq.data.length(5 * (k + 1));
      Eigen::Vector3f center = m_coordinateTransformation * j->cylinder.center;
      seq.data[5 * k + 0] = center.x();
      seq.data[5 * k + 1] = center.y();
      seq.data[5 * k + 2] = center.z();
      seq.data[5 * k + 3] = j->cylinder.radius;
      seq.data[5 * k + 4] = j->cylinder.hue;
      k++;
    }
  }
#else
  //確認のため従来の処理を実装
  size_t s = cylinders.size();
  for (size_t i = 0; i < s; i++) {
    seq.data.length(5 * (i+1));
    Eigen::Vector3f center = m_coordinateTransformation * cylinders[i].center;
    seq.data[5 * i + 0] = center.x();
    seq.data[5 * i + 1] = center.y();
    seq.data[5 * i + 2] = center.z();
    seq.data[5 * i + 3] = cylinders[i].radius;
    seq.data[5 * i + 4] = cylinders[i].hue;
  }
#endif
}


//機能：
//引数：
//戻り値： なし
void setPointCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
  pcl::PointCloud<PointType>::Ptr cloud, string name, bool colored, int r, int g, int b)
{
  if (colored) {
    pcl::visualization::PointCloudColorHandlerCustom<PointType> color(cloud, r, g, b);
    viewer->addPointCloud(cloud, color, name);
  } else {
    viewer->addPointCloud(cloud, name);
  }
}

//機能：
//引数：
//戻り値： なし
void setCube(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
  const Eigen::Vector3f &center, const Eigen::Quaternionf &rotation, float size,
  string name, int r, int g, int b)
{
  viewer->addCube(center, rotation, size, size, size, name);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r / 255.0, g / 255.0, b / 255.0, name);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, name);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
    pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name);
}

//機能：
//引数：
//戻り値： なし
void reviseCylinder(pcl::PointCloud<PointType>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients,
  Eigen::Vector3f &center, Eigen::Quaternionf &rotation, float &radius)
{
  Eigen::Vector4f centroid4f;
  pcl::compute3DCentroid(*cloud, centroid4f);
  Eigen::Vector3f centroid = centroid4f.head<3>();
  Eigen::Vector3f point;
  point <<
    coefficients->values[0],
    coefficients->values[1],
    coefficients->values[2];
  Eigen::Vector3f axis;
  axis <<
    coefficients->values[3],
    coefficients->values[4],
    coefficients->values[5];
  axis.normalize();
  center = point + (axis.dot(centroid - point))*axis;
  rotation = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitZ(), axis);
  radius = coefficients->values[6];
}

//機能：色付き点群の色相の最頻値を求める．
//引数：
//  cloud 点群
//  hbins ヒストグラムの区間数
//  smin 色相判定の対象にする彩度の最小値
//  smax 色相判定の対象にする彩度の最大値
//  vmin 色相判定の対象にする明度の最小値
//  vmax 色相判定の対象にする明度の最大値
//  title ヒストグラムを表示するOpenCVのウィンドウのタイトル．空文字ならば，ウィンドウを表示しない．
//戻り値： 色相の値（0～360の範囲），-1ならば判定不能
float calcHue(pcl::PointCloud<PointType>::ConstPtr cloud, int hbins, int smin, int smax, int vmin, int vmax, string title) {
  size_t size = cloud->points.size();
  cv::Mat bgr(1, size, CV_8UC3);
  for (size_t i = 0; i < size; i++) {
    bgr.at<cv::Vec3b>(0, i)[0] = cloud->points[i].b;
    bgr.at<cv::Vec3b>(0, i)[1] = cloud->points[i].g;
    bgr.at<cv::Vec3b>(0, i)[2] = cloud->points[i].r;
  }
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, CV_BGR2HSV);
  int channels[] = { 0 };
  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(0, smin, vmin), cv::Scalar(180, smax, vmax), mask);
  cv::Mat hist;
  int histSize[] = { hbins };
  float hranges[] = { 0, 180 };
  const float* ranges[] = { hranges };
  cv::calcHist(&hsv, 1, channels, mask, hist, 1, histSize, ranges);

  if (title != "") {
    cv::Mat hsvtable(1, hbins, CV_8UC3);
    for (int h = 0; h < hbins; h++) {
      hsvtable.at<cv::Vec3b>(0, h)[0] = hranges[0] + (hranges[1] - hranges[0])*(h + 0.5) / hbins;
      hsvtable.at<cv::Vec3b>(0, h)[1] = 255;
      hsvtable.at<cv::Vec3b>(0, h)[2] = 255;
    }
    cv::Mat bgrtable;
    cv::cvtColor(hsvtable, bgrtable, CV_HSV2BGR);
    const int hheight = 256, hwidth = 360;
    cv::Mat himg = cv::Mat::zeros(hheight, hwidth, CV_8UC3);
    int bwidth = hwidth / hbins;
    for (int h = 0; h < hbins; h++) {
      cv::Scalar color = bgrtable.at<cv::Vec3b>(0, h);
      int binVal;
      if (size == 0) {
        binVal = 0;
      } else {
        binVal = hist.at<float>(h) / size*hheight;
      }
      int x = h*bwidth;
      int y = hheight - 1;
      cv::rectangle(
        himg, cv::Point(x, y), cv::Point(x + bwidth, y - binVal),
        color, CV_FILLED);
      //cout << h << ", " << binVal << endl;
    }
    cv::imshow(title, himg);
    cv::waitKey(1);
  }
  int fmax = 0;
  int bin = -1;
  for (int h = 0; h < hbins; h++) {
    if (hist.at<float>(h) > fmax) {
      fmax = hist.at<float>(h);
      bin = h;
    }
  }
  float hue;
  if (fmax == 0) {
    hue = -1;
  } else {
    hue = 2 * (hranges[0] + (hranges[1] - hranges[0])*(bin + 0.5) / hbins); //0～360に変換
  }
  return hue;
}

//機能： HSVからRGBへの変換
//引数：
//戻り値： なし
void hsv2rgb(int h, int s, int v, int &r, int &g, int &b)
{
  cv::Mat hsv(1, 1, CV_8UC3);
  hsv.at<cv::Vec3b>(0, 0)[0] = h;
  hsv.at<cv::Vec3b>(0, 0)[1] = s;
  hsv.at<cv::Vec3b>(0, 0)[2] = v;
  cv::Mat rgb;
cv:cvtColor(hsv, rgb, CV_HSV2RGB);
  r = rgb.at<cv::Vec3b>(0, 0)[0];
  g = rgb.at<cv::Vec3b>(0, 0)[1];
  b = rgb.at<cv::Vec3b>(0, 0)[2];
}

bool checkRotationMatrix(const Eigen::Affine3f &a)
{
  Eigen::Matrix3f r = a.linear();
  const double EPS = 1e-5;
  bool check = true;
  if (abs(r.col(0).dot(r.col(0)) - 1) > EPS) { cerr << "列0の大きさが1でない" << endl; check = false; }
  if (abs(r.col(1).dot(r.col(1)) - 1) > EPS) { cerr << "列1の大きさが1でない" << endl; check = false; }
  if (abs(r.col(2).dot(r.col(2)) - 1) > EPS) { cerr << "列2の大きさが1でない" << endl; check = false; }
  if (abs(r.col(0).dot(r.col(1))) > EPS) { cerr << "列0と列1が直交していない" << endl; check = false; }
  if (abs(r.col(1).dot(r.col(2))) > EPS) { cerr << "列1と列2が直交していない" << endl; check = false; }
  if (abs(r.col(2).dot(r.col(0))) > EPS) { cerr << "列2と列0が直交していない" << endl; check = false; }
  if (abs(r.row(0).dot(r.row(0)) - 1) > EPS) { cerr << "行0の大きさが1でない" << endl; check = false; }
  if (abs(r.row(1).dot(r.row(1)) - 1) > EPS) { cerr << "行1の大きさが1でない" << endl; check = false; }
  if (abs(r.row(2).dot(r.row(2)) - 1) > EPS) { cerr << "行2の大きさが1でない" << endl; check = false; }
  if (abs(r.row(0).dot(r.row(1))) > EPS) { cerr << "行0と行1が直交していない" << endl; check = false; }
  if (abs(r.row(1).dot(r.row(2))) > EPS) { cerr << "行1と行2が直交していない" << endl; check = false; }
  if (abs(r.row(2).dot(r.row(0))) > EPS) { cerr << "行2と行0が直交していない" << endl; check = false; }
  return check;
}


extern "C"
{

  void PCToColorCylinderInit(RTC::Manager* manager)
  {
    coil::Properties profile(pctocolorcylinder_spec);
    manager->registerFactory(profile,
      RTC::Create<PCToColorCylinder>,
      RTC::Delete<PCToColorCylinder>);
  }

};


