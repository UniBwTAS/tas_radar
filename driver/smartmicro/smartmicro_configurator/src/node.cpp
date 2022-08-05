// HSDF, Philipp Berthold, philipp.berthold@unibw.de

#include "node.h"
#include "ui_node.h"


Node::Node(ros::NodeHandle& node_handle) : ros_handle_(node_handle), QMainWindow(nullptr), ui(new Ui::Node)
{
    /// Parameter
    // Topics
    node_handle.param<std::string>("topic_detectionsInput", configuration_.topic_detectionsInput, "detections");
    node_handle.param<std::string>("topic_instructionsResponseInput", configuration_.topic_instructionsResponseInput, "instructions/response");
    node_handle.param<std::string>("topic_instructionsRequestOutput", configuration_.topic_instructionsRequestOutput, "instructions/request");

    /// Subscribing & Publishing
    subscriber_detections = ros_handle_.subscribe(configuration_.topic_detectionsInput, 1, &Node::rosCallback_detections, this);
    subscriber_instructionsResponse = ros_handle_.subscribe(configuration_.topic_instructionsResponseInput, 1, &Node::rosCallback_instructions, this);
    publisher_instructionsRequest = ros_handle_.advertise<smartmicro_driver::Instructions>(configuration_.topic_instructionsRequestOutput, 1);

    /// Initialize GUI
    ui->setupUi(this);
    ui->tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    auto temp = new QTableWidgetItem();
    clearBrushOfTableWidget = temp->background();
    delete temp;    // thanks Qt

    /// Initialize Timer
    connect(&timer, SIGNAL(timeout()), this, SLOT(slotTimer()));
    timer.setInterval(200);
    timer.start();
}

Node::~Node()
{
    delete ui;
}

void Node::rosCallback_detections(const radar_msgs::DetectionRecord::ConstPtr &msg)
{
    QString frame = QString::fromStdString(msg->header.frame_id);
    bool ok;
    int ip = frame.mid(frame.lastIndexOf("/")+1).toInt(&ok);

    if (!ok || ip < 0 || ip > 254)
        return;

    bool found{false};
    for (int i = 0; i < sensors.count(); i += 1)
        if (sensors.at(i).ip == ip)
        {
            found = true;
            break;
        }
    if (!found)
    {
        Sensor sensor;
        sensor.ip = static_cast<unsigned char>(ip);
        sensors.append(sensor);
        refreshGui();
    }
}

void Node::rosCallback_instructions(const smartmicro_driver::Instructions::ConstPtr &msg)
{
    int idx = getSensorIdx(msg->destination);
    if (idx < 0)
        return;

    if (compareBlocks(msg->instructions, sensors[idx].block2012.parameters))
    {
        writeBlock(msg->instructions, sensors[idx].block2012.parameters);
        sensors[idx].block2012.last_response.restart();
    }

    if (compareBlocks(msg->instructions, sensors[idx].block2010_meas.parameters))
    {
        writeBlock(msg->instructions, sensors[idx].block2010_meas.parameters);
        sensors[idx].block2010_meas.last_response.restart();
    }

    if (compareBlocks(msg->instructions, sensors[idx].block2010_meas_132.parameters))
    {
        writeBlock(msg->instructions, sensors[idx].block2010_meas_132.parameters);
        sensors[idx].block2010_meas_132.last_response.restart();
    }

    if (compareBlocks(msg->instructions, sensors[idx].block2010_meas_153.parameters))
    {
        writeBlock(msg->instructions, sensors[idx].block2010_meas_153.parameters);
        sensors[idx].block2010_meas_153.last_response.restart();
    }

    if (compareBlocks(msg->instructions, sensors[idx].block2010_output.parameters))
    {
        writeBlock(msg->instructions, sensors[idx].block2010_output.parameters);
        sensors[idx].block2010_output.last_response.restart();
    }

    refreshGui();
}

int Node::getSensorIdx(int ip)
{
    int idx = -1;

    for (int i = 0; i < sensors.count(); i += 1)
        if (sensors.at(i).ip == ip)
        {
            idx = i;
            break;
        }

    return idx;
}

void Node::requestInstructions(quint8 ip, Node::SensorParameters &parameters)
{
    // Warning: number of instructions needs to be smaller or equal 10. Split instructions up in two or more packets, if necessary.
    if (parameters.size() > 10 || parameters.size() == 0)
        return;

    smartmicro_driver::Instructions instructions;
    instructions.destination = ip;

    instructions.instructions.reserve(parameters.size());
    for (int i = 0; i < parameters.size(); i += 1)
    {
        smartmicro_driver::Instruction in;
        in.request = parameters.at(i).request;
        in.response = smartmicro_driver::Instruction::RESPONSE_UNSET;
        in.section = parameters.at(i).section;
        in.id = parameters.at(i).id;
        in.datatype = parameters.at(i).datatype;
        in.value = parameters.at(i).value;  // only needed when sending values
        instructions.instructions.push_back(in);
    }

    publisher_instructionsRequest.publish(instructions);
}

bool Node::compareBlocks(const std::vector<smartmicro_driver::Instruction>& in, const Node::SensorParameters &parameter)
{
    if (in.size() != parameter.size())
        return false;

    for (int i = 0; i < in.size(); i += 1)
    {
        if (in.at(i).request != parameter.at(i).request)
            return false;

        if (in.at(i).section != parameter.at(i).section)
            return false;

        if (in.at(i).id != parameter.at(i).id)
            return false;

        if (in.at(i).datatype != parameter.at(i).datatype)
            return false;
    }

    return true;
}

void Node::writeBlock(const std::vector<smartmicro_driver::Instruction> &in, Node::SensorParameters &parameter)
{
    // assumes compareBlocks has already been checked

    for (int i = 0; i < in.size(); i += 1)
        parameter[i].value = in.at(i).value;
}

QString Node::parameterValueToString(const SensorParameter &parameter)
{
    if (parameter.datatype == smartmicro_driver::Instruction::DATATYPE_F32)
        return QString::number(parameter.value);
    else if (parameter.datatype == smartmicro_driver::Instruction::DATATYPE_U32)
        return intsToReadableQString(quint32(round(parameter.value)));
    else
        return intsToReadableQString(int(round(parameter.value)));
}

QString Node::intsToReadableQString(long long signed ints)
{
    bool sign = false;
    if (ints < 0)
    {
        ints = -ints;
        sign = true;
    }

    QString str = QString::number(ints);
    const int dots = int((str.count() - 1) / 3);
    const int orig_len = str.count();

    for (int i = 0; i < dots; i += 1)
        str.insert(orig_len - (1+i)*3, '.');

    if (sign)
        str.prepend("-");

    return str;
}

void Node::slotTimer()
{
    blockCounter = (blockCounter + 1) % 5;  // sensors only read first instruction packet of a burst... so enforce delays

    for (int i = 0; i < sensors.count(); i += 1)
    {
        auto& s = sensors[i];

        if ((blockCounter == 0) && ((!s.block2012.last_request.isValid()) || (s.block2012.last_request.elapsed() > 1000)))
        {
            requestInstructions(s.ip, s.block2012.parameters);
            s.block2012.last_request.restart();
        }

        if ((blockCounter == 1) && ((!s.block2010_meas.last_request.isValid()) || (s.block2010_meas.last_request.elapsed() > 1000)))
        {
            requestInstructions(s.ip, s.block2010_meas.parameters);
            s.block2010_meas.last_request.restart();
        }

        if ((blockCounter == 2) && ((!s.block2010_meas_132.last_request.isValid()) || (s.block2010_meas_132.last_request.elapsed() > 1000)))
        {
            requestInstructions(s.ip, s.block2010_meas_132.parameters);
            s.block2010_meas_132.last_request.restart();
        }

        if ((blockCounter == 3) && ((!s.block2010_meas_153.last_request.isValid()) || (s.block2010_meas_153.last_request.elapsed() > 1000)))
        {
            requestInstructions(s.ip, s.block2010_meas_153.parameters);
            s.block2010_meas_153.last_request.restart();
        }

        if ((blockCounter == 4) && ((!s.block2010_output.last_request.isValid()) || (s.block2010_output.last_request.elapsed() > 1000)))
        {
            requestInstructions(s.ip, s.block2010_output.parameters);
            s.block2010_output.last_request.restart();
        }
    }

    refreshGui();
}

void Node::refreshGui()
{
    // update tableWidget
    if (sensors.count() != ui->tableWidget->columnCount())
    {
        int oldColCount = ui->tableWidget->columnCount();
        ui->tableWidget->setColumnCount(sensors.count());

        for (int i = oldColCount; i < sensors.count(); i += 1)
        {
            auto s = sensors.at(i);

            for (int j = 0; j <= 21; j += 1)
                ui->tableWidget->setItem(j, i, new QTableWidgetItem());
            ui->tableWidget->item(0,i)->setText(QString::number(s.ip).prepend("192.168.11."));
        }
    }

    // update tableWidget items
    for (int i = 0; i < sensors.count(); i += 1)
    {
        auto& s = sensors.at(i);
        QBrush brush;

        updateGuiBlock(i, s.block2012.parameters, s.block2012.last_response);
        updateGuiBlock(i, s.block2010_meas.parameters, s.block2010_meas.last_response);
        updateGuiBlock(i, s.block2010_meas_132.parameters, s.block2010_meas_132.last_response);
        updateGuiBlock(i, s.block2010_meas_153.parameters, s.block2010_meas_153.last_response);
        updateGuiBlock(i, s.block2010_output.parameters, s.block2010_output.last_response);
    }
}

void Node::updateGuiBlock(int sensor, const Node::SensorParameters &parameters, const QElapsedTimer &timer) const
{
    QBrush br;
    if (!timer.isValid())
        br = QBrush(QColor(100,100,100));
    else if (timer.elapsed() > 2000)
        br =  QBrush(QColor("orange"));
    else
        br = clearBrushOfTableWidget;

    for (int j = 0; j < parameters.count(); j += 1)
    {
        ui->tableWidget->item(parameters.at(j).display,sensor)->setText(timer.isValid()?parameterValueToString(parameters.at(j)):"");
        ui->tableWidget->item(parameters.at(j).display,sensor)->setBackground(br);
    }
}

void Node::on_btn_txAntennaIdx_0_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 0, smartmicro_driver::Instruction::DATATYPE_U8, 0, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_txAntennaIdx_1_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 0, smartmicro_driver::Instruction::DATATYPE_U8, 1, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_txAntennaIdx_2_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 0, smartmicro_driver::Instruction::DATATYPE_U8, 2, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_rangeToggleMode_off_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 5, smartmicro_driver::Instruction::DATATYPE_U8, 0, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_rangeToggleMode_sm_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 5, smartmicro_driver::Instruction::DATATYPE_U8, 1, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_rangeToggleMode_sl_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 5, smartmicro_driver::Instruction::DATATYPE_U8, 2, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_rangeToggleMode_ml_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 5, smartmicro_driver::Instruction::DATATYPE_U8, 3, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_txRangeToggle_on_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 3, smartmicro_driver::Instruction::DATATYPE_U8, 1, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_txRangeToggle_off_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 3, smartmicro_driver::Instruction::DATATYPE_U8, 0, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_angularSeparation_on_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 4, smartmicro_driver::Instruction::DATATYPE_U8, 1, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_angularSeparation_off_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 4, smartmicro_driver::Instruction::DATATYPE_U8, 0, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_frequencySweep_short_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 2, smartmicro_driver::Instruction::DATATYPE_U8, 2, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_frequencySweep_medium_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 2, smartmicro_driver::Instruction::DATATYPE_U8, 1, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_frequencySweep_long_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 2, smartmicro_driver::Instruction::DATATYPE_U8, 0, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_destination_valueChanged(const QString &arg1)
{
    if (arg1.isEmpty())
        ui->destination->setStyleSheet("background-color: rgba(255,0,0,100);");
    else
        ui->destination->setStyleSheet("");
}

void Node::on_btn_outputCan_on_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 200, smartmicro_driver::Instruction::DATATYPE_U8, 1, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_outputCan_off_clicked()
{
    // ID 201 and ID 204 should be set to 0, too. However, they are no longer reported in the current documentation
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 200, smartmicro_driver::Instruction::DATATYPE_U8, 0, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_outputEth_on_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 202, smartmicro_driver::Instruction::DATATYPE_U8, 1, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_outputEth_off_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 202, smartmicro_driver::Instruction::DATATYPE_U8, 0, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_speedInterval_btn_clicked()
{
    SensorParameters pars;
    double value = ui->speedInterval_value->value();

    if (ui->speedInterval_negLong->isChecked())
        pars += SensorParameter{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 20, smartmicro_driver::Instruction::DATATYPE_F32, value, 0};
    if (ui->speedInterval_negMedium->isChecked())
        pars += SensorParameter{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 21, smartmicro_driver::Instruction::DATATYPE_F32, value, 0};
    if (ui->speedInterval_negShort->isChecked())
        pars += SensorParameter{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 22, smartmicro_driver::Instruction::DATATYPE_F32, value, 0};
    if (ui->speedInterval_posLong->isChecked())
        pars += SensorParameter{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 30, smartmicro_driver::Instruction::DATATYPE_F32, value, 0};
    if (ui->speedInterval_posMedium->isChecked())
        pars += SensorParameter{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 31, smartmicro_driver::Instruction::DATATYPE_F32, value, 0};
    if (ui->speedInterval_posShort->isChecked())
        pars += SensorParameter{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 32, smartmicro_driver::Instruction::DATATYPE_F32, value, 0};

    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_ip_btn_clicked()
{
    quint32 ip_raw = 3232238353 - 17 + static_cast<quint32>(ui->ip_value->value());
    double ip_m = ip_raw;   // double mantisse large enough for uint32
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 231, smartmicro_driver::Instruction::DATATYPE_U32, ip_m, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_flash_btn_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_COMMAND, 1000, 344, smartmicro_driver::Instruction::DATATYPE_U32, 2010, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}
