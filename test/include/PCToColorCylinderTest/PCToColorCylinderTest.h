// -*- C++ -*-
/*!
 * @file  PCToColorCylinderTest.h
 * @brief Extract cylinders from point cloud with color
 * @date  $Date$
 *
 * $Id$
 */

#ifndef PCTOCOLORCYLINDER_TEST__H
#define PCTOCOLORCYLINDER_TEST_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "pointcloudSVC_impl.h"
#include "BasicDataTypeSVC_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="port_stub_h">
using namespace PointCloudTypes;
// </rtc-template>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

using namespace RTC;

/*!
 * @class PCToColorCylinderTest
 * @brief Extract cylinders from point cloud with color
 *
 */
class PCToColorCylinderTest
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  PCToColorCylinderTest(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~PCToColorCylinderTest();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry() 
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   * formaer rtc_exiting_entry()
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   * former rtc_starting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   * former rtc_stopping_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * The activated action (Active state entry action)
   * former rtc_active_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   *
   * The deactivated action (Active state exit action)
   * former rtc_active_exit()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   * former rtc_active_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   * former rtc_aborting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   * former rtc_error_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   * This is same but different the former rtc_init_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * 
   * - Name:  filterXMin
   * - DefaultValue: -0.5
   */
  float m_filterXMin;
  /*!
   * 
   * - Name:  filterXMax
   * - DefaultValue: +0.5
   */
  float m_filterXMax;
  /*!
   * 
   * - Name:  filterYMin
   * - DefaultValue: -1.0
   */
  float m_filterYMin;
  /*!
   * 
   * - Name:  filterYMax
   * - DefaultValue: +1.0
   */
  float m_filterYMax;
  /*!
   * 
   * - Name:  filterZMin
   * - DefaultValue: -1.0
   */
  float m_filterZMin;
  /*!
   * 
   * - Name:  filterZMax
   * - DefaultValue: -0.4
   */
  float m_filterZMax;
  /*!
   * 
   * - Name:  leafSize
   * - DefaultValue: 0.01
   */
  float m_leafSize;
  /*!
   * 
   * - Name:  findingIterationLimit
   * - DefaultValue: 10
   */
  int m_findingIterationLimit;
  /*!
   * 
   * - Name:  segmentationMaxIteration
   * - DefaultValue: 1000
   */
  int m_segmentationMaxIteration;
  /*!
   * 
   * - Name:  segmentationDistanceThreshold
   * - DefaultValue: 0.05
   */
  float m_segmentationDistanceThreshold;
  /*!
   * 
   * - Name:  segmentationRadiusMin
   * - DefaultValue: 0.01
   */
  float m_segmentationRadiusMin;
  /*!
   * 
   * - Name:  segmentationRadiusMax
   * - DefaultValue: 0.05
   */
  float m_segmentationRadiusMax;
  /*!
   * 
   * - Name:  cylinderPointSizeMin
   * - DefaultValue: 100
   */
  int m_cylinderPointSizeMin;
  /*!
   * 
   * - Name:  displayHistogram
   * - DefaultValue: 1
   */
  int m_displayHistogram;
  /*!
   * 
   * - Name:  histogramBinNumber
   * - DefaultValue: 16
   */
  int m_histogramBinNumber;
  /*!
   * 
   * - Name:  saturationMin
   * - DefaultValue: 127
   */
  int m_saturationMin;
  /*!
   * 
   * - Name:  saturationMax
   * - DefaultValue: 255
   */
  int m_saturationMax;
  /*!
   * 
   * - Name:  valueMin
   * - DefaultValue: 0
   */
  int m_valueMin;
  /*!
   * 
   * - Name:  valueMax
   * - DefaultValue: 255
   */
  int m_valueMax;
  /*!
   * 
   * - Name:  sameCylinderCenterDistanceLimit
   * - DefaultValue: 0.02
   */
  float m_sameCylinderCenterDistanceLimit;
  /*!
   * 
   * - Name:  sameCylinderRadiusDistanceLimit
   * - DefaultValue: 0.01
   */
  float m_sameCylinderRadiusDistanceLimit;
  /*!
   * 
   * - Name:  sameCylinderHueDistanceLimit
   * - DefaultValue: 23
   */
  float m_sameCylinderHueDistanceLimit;
  /*!
   * 
   * - Name:  cylinderAccumulationMin
   * - DefaultValue: 10
   */
  int m_cylinderAccumulationMin;
  /*!
   * 
   * - Name:  coordinateTransformationFile
   * - DefaultValue: coordinateTransformation.txt
   */
  std::string m_coordinateTransformationFile;
  /*!
   * 
   * - Name:  calibrationOffsetX
   * - DefaultValue: 0.0
   */
  float m_calibrationOffsetX;
  /*!
   * 
   * - Name:  calibrationOffsetY
   * - DefaultValue: 0.0
   */
  float m_calibrationOffsetY;
  /*!
   * 
   * - Name:  calibrationOffsetZ
   * - DefaultValue: 0.0
   */
  float m_calibrationOffsetZ;

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  RTC::TimedDoubleSeq m_cylinder;
  /*!
   */
  InPort<RTC::TimedDoubleSeq> m_cylinderIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  PointCloudTypes::PointCloud m_pc;
  /*!
   */
  OutPort<PointCloudTypes::PointCloud> m_pcOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>

};


extern "C"
{
  DLL_EXPORT void PCToColorCylinderTestInit(RTC::Manager* manager);
};

#endif // PCTOCOLORCYLINDER_TEST_H
