// HSDF, Philipp Berthold, philipp.berthold@unibw.de

#pragma once

#include <ros/ros.h>
#include <QMainWindow>
#include <QTimer>
#include <smartmicro_driver/Instructions.h>
#include <smartmicro_driver/Sensors.h>
#include <smartmicro_driver/Sensor.h>
#include <QElapsedTimer>
#include <QMap>
#include <math.h>


#define IP                      0
#define RADAR_TYPE_DISPLAY      1

#define INTERFACE_VERSION_MAJOR 2
#define INTERFACE_VERSION_MINOR 3
#define SOFTWARE_GENERATION     4
#define SOFTWARE_MAJOR          5
#define SOFTWARE_MINOR          6
#define SOFTWARE_PATCH          7
#define SERIAL                  8
#define UPTIME_LOW              9
#define UPTIME_HIGH             10

#define TX_ANTENNA_IDX          11
#define CENTER_FREQUENCY_IDX    12
#define FREQUENCY_SWEEP_IDX     13

#define ENABLE_TX_ANT_TOGGLE    14
#define ANGULAR_SEPARATION      15

#define RANGE_TOGGLE_MODE       16
#define SPEED_NEG_LONG          17
#define SPEED_NEG_MID           18
#define SPEED_NEG_SHORT         19
#define SPEED_POS_LONG          20
#define SPEED_POS_MID           21
#define SPEED_POS_SHORT         22

#define OUTPUT_CAN              23
#define OUTPUT_ETH              24
#define IP_SOURCE_ADDRESS       25

#define DRVEGRD_SIGNATURE_STATUS_IF_VERSION_MAJOR   1117233612u
#define DRVEGRD_SIGNATURE_STATUS_IF_VERSION_MINOR   1033132100u
#define DRVEGRD_SIGNATURE_STATUS_SW_GENERATION      2333096865u
#define DRVEGRD_SIGNATURE_STATUS_SW_MAJOR           3596596704u
#define DRVEGRD_SIGNATURE_STATUS_SW_MINOR           352653743u
#define DRVEGRD_SIGNATURE_STATUS_SW_PATCH           2609227558u

#define DRVEGRD_SIGNATURE_PARAM_CENTER_FREQUENCY    2498156403u
#define DRVEGRD_SIGNATURE_PARAM_FREQUENCY_SWEEP     3338708177u
#define DRVEGRD_SIGNATURE_PARAM_HIRES_MODE          2742613544u
#define DRVEGRD_SIGNATURE_PARAM_TARGET_VALIDATION   3438780219u
#define DRVEGRD_SIGNATURE_PARAM_OUTPUT_ETH          2473833729u
#define DRVEGRD_SIGNATURE_PARAM_IP_SOURCE           1602849932u
#define DRVEGRD_SIGNATURE_PARAM_SUBNET_MASK         1475187331u
#define DRVEGRD_SIGNATURE_PARAM_IP_DEST             4037333986u
#define DRVEGRD_SIGNATURE_PARAM_IP_DEST_PORT        1493289235u

#define DRVEGRD_SIGNATURE_COMMAND_REBOOT            3120196712u
#define DRVEGRD_SIGNATURE_COMMAND_RESET_ALL         3022381235u
#define DRVEGRD_SIGNATURE_COMMAND_RESET_KEEP_IP     3308040184u

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
    ros::Subscriber subscriber_sensors;
    ros::Subscriber subscriber_instructionsResponse;
    ros::Publisher 	publisher_instructionsRequest;

private:
    // ROS data reception callbacks
    void rosCallback_sensors(const smartmicro_driver::Sensors::ConstPtr &msg);
    void rosCallback_instructions(const smartmicro_driver::Instructions::ConstPtr &msg);

private:
    // configuration
    struct
    {
        // ROS topics
        std::string topic_instructionsResponseInput;
        std::string topic_sensorsInput;
        std::string topic_instructionsRequestOutput;

    }   configuration_;

private:
    QTimer timer;
    QBrush clearBrushOfTableWidget;
    QBrush clearTextBrushOfTableWidget;

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
        SensorParameter(
                quint8 request_in = 0,
                quint16 section_in = 0,
                quint16 id_in = 0,
                quint8 datatype_in = 0,
                quint32 signature_in = 0,
                double value_in = 0.0,
                quint8 display_in = 0,
                quint8 last_response_in = smartmicro_driver::Instruction::RESPONSE_UNSET)
            : request(request_in)
            , section(section_in)
            , id(id_in)
            , datatype(datatype_in)
            , signature(signature_in)
            , value(value_in)
            , display(display_in)
            , last_response(last_response_in)
        {
        }

        quint8 request;
        quint16 section;
        quint16 id;
        quint8 datatype;
        quint32 signature;
        double value;
        quint8 display;
        quint8 last_response;
    };

    typedef QVector<SensorParameter> SensorParameters;

    // Sensor Parameter Blocks for atomic read

    struct SensorBlock_2012
    {
        QElapsedTimer last_request;
        QElapsedTimer last_response;

        SensorParameters parameters{
          {smartmicro_driver::Instruction::REQUEST_STATUS_GET, 2012, 2,  smartmicro_driver::Instruction::DATATYPE_U16,0,0,SOFTWARE_GENERATION},
          {smartmicro_driver::Instruction::REQUEST_STATUS_GET, 2012, 3,  smartmicro_driver::Instruction::DATATYPE_U16,0,0,SOFTWARE_MAJOR},
          {smartmicro_driver::Instruction::REQUEST_STATUS_GET, 2012, 4,  smartmicro_driver::Instruction::DATATYPE_U16,0,0,SOFTWARE_MINOR},
          {smartmicro_driver::Instruction::REQUEST_STATUS_GET, 2012, 5,  smartmicro_driver::Instruction::DATATYPE_U16,0,0,SOFTWARE_PATCH},
          {smartmicro_driver::Instruction::REQUEST_STATUS_GET, 2012, 9,  smartmicro_driver::Instruction::DATATYPE_U32,0,0,SERIAL},
          {smartmicro_driver::Instruction::REQUEST_STATUS_GET, 2012, 20, smartmicro_driver::Instruction::DATATYPE_U32,0,0,UPTIME_LOW},
          {smartmicro_driver::Instruction::REQUEST_STATUS_GET, 2012, 21, smartmicro_driver::Instruction::DATATYPE_U32,0,0,UPTIME_HIGH},
        };
    };

    struct SensorBlock_7012
    {
        QElapsedTimer last_request;
        QElapsedTimer last_response;

        SensorParameters parameters{
          {smartmicro_driver::Instruction::REQUEST_STATUS_GET, 7012, 2, smartmicro_driver::Instruction::DATATYPE_U16, DRVEGRD_SIGNATURE_STATUS_SW_GENERATION, 0, SOFTWARE_GENERATION},
          {smartmicro_driver::Instruction::REQUEST_STATUS_GET, 7012, 3, smartmicro_driver::Instruction::DATATYPE_U16, DRVEGRD_SIGNATURE_STATUS_SW_MAJOR, 0, SOFTWARE_MAJOR},
          {smartmicro_driver::Instruction::REQUEST_STATUS_GET, 7012, 4, smartmicro_driver::Instruction::DATATYPE_U16, DRVEGRD_SIGNATURE_STATUS_SW_MINOR, 0, SOFTWARE_MINOR},
          {smartmicro_driver::Instruction::REQUEST_STATUS_GET, 7012, 5, smartmicro_driver::Instruction::DATATYPE_U16, DRVEGRD_SIGNATURE_STATUS_SW_PATCH, 0, SOFTWARE_PATCH},
          {smartmicro_driver::Instruction::REQUEST_STATUS_GET, 7012, 0, smartmicro_driver::Instruction::DATATYPE_U32, DRVEGRD_SIGNATURE_STATUS_IF_VERSION_MAJOR, 0, INTERFACE_VERSION_MAJOR},
          {smartmicro_driver::Instruction::REQUEST_STATUS_GET, 7012, 1, smartmicro_driver::Instruction::DATATYPE_U32, DRVEGRD_SIGNATURE_STATUS_IF_VERSION_MINOR, 0, INTERFACE_VERSION_MINOR},
        };
    };

    struct SensorBlock_2010_meas
    {
        QElapsedTimer last_request;
        QElapsedTimer last_response;

        SensorParameters parameters{
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 0,  smartmicro_driver::Instruction::DATATYPE_U8,0,0,TX_ANTENNA_IDX},
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 1,  smartmicro_driver::Instruction::DATATYPE_U8,0,0,CENTER_FREQUENCY_IDX},
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 2,  smartmicro_driver::Instruction::DATATYPE_U8,0,0,FREQUENCY_SWEEP_IDX},
        };
    };

    struct SensorBlock_7010_meas
    {
        QElapsedTimer last_request;
        QElapsedTimer last_response;

        SensorParameters parameters{
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 7010, 1,  smartmicro_driver::Instruction::DATATYPE_U8, DRVEGRD_SIGNATURE_PARAM_CENTER_FREQUENCY, 0, CENTER_FREQUENCY_IDX},
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 7010, 2,  smartmicro_driver::Instruction::DATATYPE_U8, DRVEGRD_SIGNATURE_PARAM_FREQUENCY_SWEEP, 0, FREQUENCY_SWEEP_IDX},
        };
    };

    struct SensorBlock_2010_meas_132
    {
        QElapsedTimer last_request;
        QElapsedTimer last_response;

        SensorParameters parameters{
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 3,  smartmicro_driver::Instruction::DATATYPE_U8,0,0,ENABLE_TX_ANT_TOGGLE},
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 4,  smartmicro_driver::Instruction::DATATYPE_U8,0,0,ANGULAR_SEPARATION},
        };
    };

    struct SensorBlock_7010_modes
    {
        QElapsedTimer last_request;
        QElapsedTimer last_response;

        SensorParameters parameters{
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 7010, 10,  smartmicro_driver::Instruction::DATATYPE_U8, DRVEGRD_SIGNATURE_PARAM_HIRES_MODE, 0, ENABLE_TX_ANT_TOGGLE},
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 7010, 11,  smartmicro_driver::Instruction::DATATYPE_U8, DRVEGRD_SIGNATURE_PARAM_TARGET_VALIDATION, 0, ANGULAR_SEPARATION},
        };
    };

    struct SensorBlock_2010_meas_153
    {
        QElapsedTimer last_request;
        QElapsedTimer last_response;

        SensorParameters parameters{
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 5,  smartmicro_driver::Instruction::DATATYPE_U8,0,0,RANGE_TOGGLE_MODE},
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 20, smartmicro_driver::Instruction::DATATYPE_F32,0,0,SPEED_NEG_LONG},
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 21, smartmicro_driver::Instruction::DATATYPE_F32,0,0,SPEED_NEG_MID},
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 22, smartmicro_driver::Instruction::DATATYPE_F32,0,0,SPEED_NEG_SHORT},
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 30, smartmicro_driver::Instruction::DATATYPE_F32,0,0,SPEED_POS_LONG},
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 31, smartmicro_driver::Instruction::DATATYPE_F32,0,0,SPEED_POS_MID},
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 32, smartmicro_driver::Instruction::DATATYPE_F32,0,0,SPEED_POS_SHORT},
        };
    };

    struct SensorBlock_2010_output
    {
        QElapsedTimer last_request;
        QElapsedTimer last_response;

        SensorParameters parameters{
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 200, smartmicro_driver::Instruction::DATATYPE_U8,0,0,OUTPUT_CAN},   // Target List via CAN
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 202, smartmicro_driver::Instruction::DATATYPE_U8,0,0,OUTPUT_ETH},   // Target List via Eth
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 2010, 231, smartmicro_driver::Instruction::DATATYPE_U32,0,0,IP_SOURCE_ADDRESS},
        };
    };

    struct SensorBlock_7010_output
    {
        QElapsedTimer last_request;
        QElapsedTimer last_response;

        SensorParameters parameters{
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 7010, 202, smartmicro_driver::Instruction::DATATYPE_U8, DRVEGRD_SIGNATURE_PARAM_OUTPUT_ETH, 0, OUTPUT_ETH},
        };
    };

    struct SensorBlock_7010_ip
    {
        QElapsedTimer last_request;
        QElapsedTimer last_response;

        SensorParameters parameters{
          {smartmicro_driver::Instruction::REQUEST_PARAMETER_GET, 7010, 231, smartmicro_driver::Instruction::DATATYPE_U32, DRVEGRD_SIGNATURE_PARAM_IP_SOURCE, 0, IP_SOURCE_ADDRESS},
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
        quint8 radar_type{smartmicro_driver::Sensor::RADAR_TYPE_UNKNOWN};
        SensorBlock_2012 block2012;
        SensorBlock_7012 block7012;
        SensorBlock_2010_meas block2010_meas;
        SensorBlock_7010_meas block7010_meas;
        SensorBlock_2010_meas_132 block2010_meas_132;
        SensorBlock_7010_modes block7010_modes;
        SensorBlock_2010_meas_153 block2010_meas_153;
        SensorBlock_2010_output block2010_output;
        SensorBlock_7010_output block7010_output;
        SensorBlock_7010_ip block7010_ip;
    };
    QVector<Sensor> sensors;
    int getSensorIdx(int ip);
    static bool isDrvegrd(quint8 radar_type);
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
    void on_btn_centerFrequency_0_drvegrd_clicked();
    void on_btn_centerFrequency_1_drvegrd_clicked();
    void on_btn_centerFrequency_2_drvegrd_clicked();
    void on_btn_centerFrequency_3_drvegrd_clicked();
    void on_btn_frequencySweep_short_drvegrd_clicked();
    void on_btn_frequencySweep_medium_drvegrd_clicked();
    void on_btn_frequencySweep_long_drvegrd_clicked();
    void on_btn_hires_on_drvegrd_clicked();
    void on_btn_hires_off_drvegrd_clicked();
    void on_btn_targetValidation_on_drvegrd_clicked();
    void on_btn_targetValidation_off_drvegrd_clicked();
    void on_btn_outputEth_on_drvegrd_clicked();
    void on_btn_outputEth_off_drvegrd_clicked();
    void on_ip_btn_drvegrd_clicked();
    void on_flash_btn_drvegrd_clicked();
    void on_tableWidget_cellClicked(int row, int column);

private:
    void updateGuiBlock(int sensor, SensorParameters const& parameters, QElapsedTimer const& timer) const;
    void refreshControlAvailability();

};
