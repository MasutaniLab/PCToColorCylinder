// -*- C++ -*-
/*!
 * @file  PCToColorCylinderTest.cpp
 * @brief Extract cylinders from point cloud with color
 * @date $Date$
 *
 * $Id$
 */

#include "PCToColorCylinderTest.h"

// Module specification
// <rtc-template block="module_spec">
static const char* pctocolorcylinder_spec[] =
  {
    "implementation_id", "PCToColorCylinderTest",
    "type_name",         "PCToColorCylinderTest",
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
PCToColorCylinderTest::PCToColorCylinderTest(RTC::Manager* manager)
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
PCToColorCylinderTest::~PCToColorCylinderTest()
{
}



RTC::ReturnCode_t PCToColorCylinderTest::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("cylinder", m_cylinderIn);
  
  // Set OutPort buffer
  addOutPort("pc", m_pcOut);
  
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
RTC::ReturnCode_t PCToColorCylinderTest::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PCToColorCylinderTest::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PCToColorCylinderTest::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t PCToColorCylinderTest::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t PCToColorCylinderTest::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t PCToColorCylinderTest::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t PCToColorCylinderTest::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PCToColorCylinderTest::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PCToColorCylinderTest::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PCToColorCylinderTest::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PCToColorCylinderTest::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void PCToColorCylinderTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(pctocolorcylinder_spec);
    manager->registerFactory(profile,
                             RTC::Create<PCToColorCylinderTest>,
                             RTC::Delete<PCToColorCylinderTest>);
  }
  
};


