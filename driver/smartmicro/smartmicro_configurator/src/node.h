// HSDF, Philipp Berthold, philipp.berthold@unibw.de

#pragma once

#include <ros/ros.h>
#include <QMainWindow>
#include <QTimer>
#include <radar_msgs/DetectionRecord.h>
#include <smartmicro_driver/Instructions.h>
#include <QElapsedTimer>
#include <QMap>
#include <math.h>


#define IP                      0

#define SOFTWARE_GENERATION     1
#define SOFTWARE_MAJOR          2
#define SOFTWARE_MINOR          3
#define SOFTWARE_PATCH          4
#define SERIAL                  5
#define UPTIME_LOW              6
#define UPTIME_HIGH             7

#define TX_ANTENNA_IDX          8
#define CENTER_FREQUENCY_IDX    9
#define FREQUENCY_SWEEP_IDX     10

#define ENABLE_TX_ANT_TOGGLE    11
#define ANGULAR_SEPARATION      12

#define RANGE_TOGGLE_MODE       13
#define SPEED_NEG_LONG          14
#define SPEED_NEG_MID           15
#define SPEED_NEG_SHORT         16
#define SPEED_POS_LONG          17
#define SPEED_POS_MID           18
#define SPEED_POS_SHORT         19

#define OUTPUT_CAN              20
#define OUTPUT_ETH              21

namespace Ui {
class Node;
}

class Node : public QMainWindow
{
    Q_OBJECT

public:
    explicit Node(ros::NodeHandle& node_handle);
    ~Node();

private:
    Ui::Node *ui;

private:
    // Reference to ROS Handle
    ros::NodeHandle& ros_handle_;

    // ROS Interfaces
    ros::Subscriber subscriber_detections;
    ros::Subscriber subscriber_instructionsResponse;
    ros::Publisher 	publisher_instructionsRequest;

private:
    // ROS data reception callbacks
    void rosCallback_detections(const radar_msgs::DetectionRecord::ConstPtr &msg);
    void rosCallback_instructions(const smartmicro_driver::Instructions::ConstPtr &msg);

private:
    // configuration
    struct
    {
        // ROS topics
        std::string topic_instructionsResponseInput;
        std::string topic_detectionsInput;
        std::string topic_instructionsRequestOutput;

    }   configuration_;

private:
    QTimer timer;
    QBrush clearBrushOfTableWidget;

private:
    //template<typename T>
    struct SensorParameter2
    {
        //T value;
        QString value;
        QElapsedTimer last_request;
        QElapsedTimer last_response;
    };

    struct SensorParameter
    {
        quint8 request;
        quint16 section;
        quint16 id;
        quint8 datatype;
        double value;
        quint8 display;
    };

    typedef QVector<SensorParameter> SensorParameters;

    // Sensor Parameter Blocks for atomic read

    struct SensorBlock_2012
    {
        QElapsedTimer last_request;
        QElapsedTimer last_response;

        SensorParameters parameters{
          {smartmicro_driver::Instruction::REQUEST_STATUS_GET, 2012, 2,  smartmicro_driver::Instruction::DATATYPE_U16,0,SOFTWARE_GENERATION},
          {smartmicro_driver::Instruction::REQUEST_STATUS_GET, 2012, 3,  smartmicro_driver::Instruction::DATATYPE_U16,0,SOFTWARE_MAJOR},
          {smartmicro_driver::Instruction::REQUEST_STATUS_GET, 2012, 3,  smartmicro_driver::Instruction::DATATYPE_U16,0,SOFTWARE_MINOR},
          {smartmicro_driver::Instruction::REQUEST_STATUS_GET, 2012, 5,  smartmicro_driver::Instruction::DATATYPE_U16,0,SOFTWARE_PATCH},
          {smartmicro_driver::Instruction::REQUEST_STATUS_GET, 2012, 9,  smartmicro_driver::Instruction::DATATYPE_U32,0,SERIAL},
          {smartmicro_driver::Instruction::REQUEST_STATUS_GET, 2012, 20, smartmicro_driver::Instruction::DATATYPE_U32,0,UPTIME_LOW},
          {smartmicro_driver::Instruction::REQUEST_STATUS_GET, 2012, 21, smartmicro_driver::Instruction::DATATYPE_U32,0,UPTIME_HIGH},
        };
    };

    struct SensorBlock_2010_meas
    {
        QElapsedTimer last_request;
        QElapsedTimer last_response;

        SensorParameters parameters{
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 0,  smartmicro_driver::Instruction::DATATYPE_U8,0,TX_ANTENNA_IDX},
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 1,  smartmicro_driver::Instruction::DATATYPE_U8,0,CENTER_FREQUENCY_IDX},
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 2,  smartmicro_driver::Instruction::DATATYPE_U8,0,FREQUENCY_SWEEP_IDX},
        };
    };

    struct SensorBlock_2010_meas_132
    {
        QElapsedTimer last_request;
        QElapsedTimer last_response;

        SensorParameters parameters{
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 3,  smartmicro_driver::Instruction::DATATYPE_U8,0,ENABLE_TX_ANT_TOGGLE},
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 4,  smartmicro_driver::Instruction::DATATYPE_U8,0,ANGULAR_SEPARATION},
        };
    };

    struct SensorBlock_2010_meas_153
    {
        QElapsedTimer last_request;
        QElapsedTimer last_response;

        SensorParameters parameters{
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 5,  smartmicro_driver::Instruction::DATATYPE_U8,0,RANGE_TOGGLE_MODE},
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 20, smartmicro_driver::Instruction::DATATYPE_F32,0,SPEED_NEG_LONG},
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 21, smartmicro_driver::Instruction::DATATYPE_F32,0,SPEED_NEG_MID},
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 22, smartmicro_driver::Instruction::DATATYPE_F32,0,SPEED_NEG_SHORT},
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 30, smartmicro_driver::Instruction::DATATYPE_F32,0,SPEED_POS_LONG},
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 31, smartmicro_driver::Instruction::DATATYPE_F32,0,SPEED_POS_MID},
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 32, smartmicro_driver::Instruction::DATATYPE_F32,0,SPEED_POS_SHORT},
        };
    };

    struct SensorBlock_2010_output
    {
        QElapsedTimer last_request;
        QElapsedTimer last_response;

        SensorParameters parameters{
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 200, smartmicro_driver::Instruction::DATATYPE_U8,0,OUTPUT_CAN},   // Target List via CAN
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 202, smartmicro_driver::Instruction::DATATYPE_U8,0,OUTPUT_ETH},   // Target List via Eth
        };
    };

    struct SensorBlock_2012_output
    {
        QElapsedTimer last_request;
        QElapsedTimer last_response;

        SensorParameter outputCan;
        SensorParameter outputEth;
    };

    struct Sensor
    {
        quint8 ip;
        SensorBlock_2012 block2012;
        SensorBlock_2010_meas block2010_meas;
        SensorBlock_2010_meas_132 block2010_meas_132;
        SensorBlock_2010_meas_153 block2010_meas_153;
        SensorBlock_2010_output block2010_output;
    };
    QVector<Sensor> sensors;
    int getSensorIdx(int ip);
    void requestInstructions(quint8 ip, SensorParameters &parameters);
    bool compareBlocks(const std::vector<smartmicro_driver::Instruction>& in, SensorParameters const& parameter);
    void writeBlock(const std::vector<smartmicro_driver::Instruction>& in, SensorParameters &parameter);
    int blockCounter{0};
    static QString parameterValueToString(SensorParameter const& parameter);
    static QString intsToReadableQString(signed long long ints);

private slots:
    // Qt Slots
    void slotTimer();
    void refreshGui();

    void on_btn_txAntennaIdx_0_clicked();

    void on_btn_txAntennaIdx_1_clicked();

    void on_btn_txAntennaIdx_2_clicked();

    void on_btn_rangeToggleMode_off_clicked();

    void on_btn_rangeToggleMode_sm_clicked();

    void on_btn_rangeToggleMode_sl_clicked();

    void on_btn_rangeToggleMode_ml_clicked();

    void on_btn_txRangeToggle_on_clicked();

    void on_btn_txRangeToggle_off_clicked();

    void on_btn_angularSeparation_on_clicked();

    void on_btn_angularSeparation_off_clicked();

    void on_btn_frequencySweep_short_clicked();

    void on_btn_frequencySweep_medium_clicked();

    void on_btn_frequencySweep_long_clicked();

    void on_destination_valueChanged(const QString &arg1);

    void on_btn_outputCan_on_clicked();

    void on_btn_outputCan_off_clicked();

    void on_btn_outputEth_on_clicked();

    void on_btn_outputEth_off_clicked();

    void on_speedInterval_btn_clicked();

    void on_ip_btn_clicked();

    void on_flash_btn_clicked();

private:
    void updateGuiBlock(int sensor, SensorParameters const& parameters, QElapsedTimer const& timer) const;

};
