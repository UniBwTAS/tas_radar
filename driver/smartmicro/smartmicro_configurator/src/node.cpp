// HSDF, Philipp Berthold, philipp.berthold@unibw.de

#include "node.h"
#include "ui_node.h"


Node::Node(ros::NodeHandle& node_handle) : ros_handle_(node_handle), QMainWindow(nullptr), ui(new Ui::Node)
{
    ros::NodeHandle private_nh("~");

    /// Parameter
    // Topics
    private_nh.param<std::string>("topic_sensorsInput", configuration_.topic_sensorsInput, "sensors");
    private_nh.param<std::string>("topic_instructionsResponseInput", configuration_.topic_instructionsResponseInput, "instructions/response");
    private_nh.param<std::string>("topic_instructionsRequestOutput", configuration_.topic_instructionsRequestOutput, "instructions/request");

    /// Subscribing & Publishing
    subscriber_sensors = ros_handle_.subscribe(configuration_.topic_sensorsInput, 1, &Node::rosCallback_sensors, this);
    subscriber_instructionsResponse = ros_handle_.subscribe(configuration_.topic_instructionsResponseInput, 1, &Node::rosCallback_instructions, this);
    publisher_instructionsRequest = ros_handle_.advertise<smartmicro_driver::Instructions>(configuration_.topic_instructionsRequestOutput, 1);

    /// Initialize GUI
    ui->setupUi(this);
    ui->tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    auto temp = new QTableWidgetItem();
    clearBrushOfTableWidget = temp->background();
    clearTextBrushOfTableWidget = temp->foreground();
    delete temp;    // thanks Qt

    /// Initialize Timer
    connect(&timer, SIGNAL(timeout()), this, SLOT(slotTimer()));
    timer.setInterval(200);
    timer.start();

    refreshControlAvailability();
}

Node::~Node()
{
    delete ui;
}

bool Node::isDrvegrd(quint8 radar_type)
{
    return radar_type == smartmicro_driver::Sensor::RADAR_TYPE_DRVEGRD;
}

void Node::rosCallback_sensors(const smartmicro_driver::Sensors::ConstPtr &msg)
{
    bool sensors_changed = false;

    for (auto const& sensor_msg : msg->sensors)
    {
        int ip = sensor_msg.ip;
        if (ip < 0 || ip > 254)
            continue;

        bool found{false};
        for (int i = 0; i < sensors.count(); i += 1)
        {
            if (sensors.at(i).ip == ip)
            {
                found = true;
                if (sensors[i].radar_type != sensor_msg.radar_type)
                {
                    sensors[i].radar_type = sensor_msg.radar_type;
                    sensors_changed = true;
                }
                break;
            }
        }

        if (!found)
        {
            Sensor sensor;
            sensor.ip = static_cast<unsigned char>(ip);
            sensor.radar_type = sensor_msg.radar_type;
            sensors.append(sensor);
            if (ui->destination->value() == 0)
            {
                ui->destination->setValue(ip);
                ui->ip_value->setValue(ip);
                ui->ip_value_drvegrd->setValue(ip);
            }
            sensors_changed = true;
        }
    }

    if (sensors_changed)
    {
        refreshGui();
        refreshControlAvailability();
    }
}

void Node::rosCallback_instructions(const smartmicro_driver::Instructions::ConstPtr &msg)
{
    auto inFlightIt = inFlightRequests_.find(msg->destination);
    if (inFlightIt != inFlightRequests_.end() && compareBlocks(msg->instructions, inFlightIt->request.parameters))
        inFlightRequests_.erase(inFlightIt);

    int idx = getSensorIdx(msg->destination);
    if (idx < 0)
        return;

    if (compareBlocks(msg->instructions, sensors[idx].block2012.parameters))
    {
        writeBlock(msg->instructions, sensors[idx].block2012.parameters);
        sensors[idx].block2012.last_response.restart();
    }

    if (compareBlocks(msg->instructions, sensors[idx].block7012.parameters))
    {
        writeBlock(msg->instructions, sensors[idx].block7012.parameters);
        sensors[idx].block7012.last_response.restart();
    }

    if (compareBlocks(msg->instructions, sensors[idx].block2010_meas.parameters))
    {
        writeBlock(msg->instructions, sensors[idx].block2010_meas.parameters);
        sensors[idx].block2010_meas.last_response.restart();
    }

    if (compareBlocks(msg->instructions, sensors[idx].block7010_meas.parameters))
    {
        writeBlock(msg->instructions, sensors[idx].block7010_meas.parameters);
        sensors[idx].block7010_meas.last_response.restart();
    }

    if (compareBlocks(msg->instructions, sensors[idx].block2010_meas_132.parameters))
    {
        writeBlock(msg->instructions, sensors[idx].block2010_meas_132.parameters);
        sensors[idx].block2010_meas_132.last_response.restart();
    }

    if (compareBlocks(msg->instructions, sensors[idx].block7010_modes.parameters))
    {
        writeBlock(msg->instructions, sensors[idx].block7010_modes.parameters);
        sensors[idx].block7010_modes.last_response.restart();
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

    if (compareBlocks(msg->instructions, sensors[idx].block7010_output.parameters))
    {
        writeBlock(msg->instructions, sensors[idx].block7010_output.parameters);
        sensors[idx].block7010_output.last_response.restart();
    }

    if (compareBlocks(msg->instructions, sensors[idx].block7010_ip.parameters))
    {
        writeBlock(msg->instructions, sensors[idx].block7010_ip.parameters);
        sensors[idx].block7010_ip.last_response.restart();
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

    QueuedRequest request;
    request.ip = ip;
    request.parameters = parameters;
    request.key = makeRequestKey(ip, parameters);
    request.high_priority = true;

    for (auto it = requestQueue_.begin(); it != requestQueue_.end(); ++it)
    {
        if (it->key == request.key)
        {
            *it = request;
            processInstructionQueue();
            return;
        }
    }

    requestQueue_.push_front(request);
    processInstructionQueue();
}

void Node::enqueuePollingRequest(quint8 ip, const Node::SensorParameters &parameters)
{
    if (parameters.size() > 10 || parameters.size() == 0)
        return;

    if (isInFlightRequest(ip, parameters))
        return;

    QueuedRequest request;
    request.ip = ip;
    request.parameters = parameters;
    request.key = makeRequestKey(ip, parameters);

    for (auto const& queued : requestQueue_)
        if (queued.key == request.key)
            return;

    requestQueue_.push_back(request);
}

QString Node::makeRequestKey(quint8 ip, const Node::SensorParameters &parameters) const
{
    QString key = QString::number(ip);
    for (auto const& parameter : parameters)
    {
        key += QString("|%1:%2:%3:%4:%5")
                .arg(parameter.request)
                .arg(parameter.section)
                .arg(parameter.id)
                .arg(parameter.datatype)
                .arg(parameter.signature);
    }
    return key;
}

void Node::markBlockRequestSent(quint8 ip, const Node::SensorParameters &parameters)
{
    int const idx = getSensorIdx(ip);
    if (idx < 0)
        return;

    auto& sensor = sensors[idx];

    if (compareParameterBlocks(parameters, sensor.block2012.parameters))
        sensor.block2012.last_request.restart();
    else if (compareParameterBlocks(parameters, sensor.block7012.parameters))
        sensor.block7012.last_request.restart();
    else if (compareParameterBlocks(parameters, sensor.block2010_meas.parameters))
        sensor.block2010_meas.last_request.restart();
    else if (compareParameterBlocks(parameters, sensor.block7010_meas.parameters))
        sensor.block7010_meas.last_request.restart();
    else if (compareParameterBlocks(parameters, sensor.block2010_meas_132.parameters))
        sensor.block2010_meas_132.last_request.restart();
    else if (compareParameterBlocks(parameters, sensor.block7010_modes.parameters))
        sensor.block7010_modes.last_request.restart();
    else if (compareParameterBlocks(parameters, sensor.block2010_meas_153.parameters))
        sensor.block2010_meas_153.last_request.restart();
    else if (compareParameterBlocks(parameters, sensor.block2010_output.parameters))
        sensor.block2010_output.last_request.restart();
    else if (compareParameterBlocks(parameters, sensor.block7010_output.parameters))
        sensor.block7010_output.last_request.restart();
    else if (compareParameterBlocks(parameters, sensor.block7010_ip.parameters))
        sensor.block7010_ip.last_request.restart();
}

bool Node::isInFlightRequest(quint8 ip, const Node::SensorParameters &parameters) const
{
    auto it = inFlightRequests_.find(ip);
    return (it != inFlightRequests_.end()) && compareParameterBlocks(parameters, it->request.parameters);
}

void Node::processInstructionQueue()
{
    for (auto it = inFlightRequests_.begin(); it != inFlightRequests_.end(); )
    {
        if (it->since.isValid() && it->since.elapsed() > requestTimeoutMs_)
            it = inFlightRequests_.erase(it);
        else
            ++it;
    }

    if (requestQueue_.isEmpty())
        return;

    if (lastSendTime_.isValid() && lastSendTime_.elapsed() < requestSendSpacingMs_)
        return;

    int requestIndex = -1;
    for (int i = 0; i < requestQueue_.size(); i += 1)
    {
        if (!inFlightRequests_.contains(requestQueue_.at(i).ip))
        {
            requestIndex = i;
            break;
        }
    }

    if (requestIndex < 0)
        return;

    smartmicro_driver::Instructions instructions;
    auto request = requestQueue_.takeAt(requestIndex);
    instructions.destination = request.ip;

    instructions.instructions.reserve(request.parameters.size());
    for (int i = 0; i < request.parameters.size(); i += 1)
    {
        smartmicro_driver::Instruction in;
        in.request = request.parameters.at(i).request;
        in.response = smartmicro_driver::Instruction::RESPONSE_UNSET;
        in.section = request.parameters.at(i).section;
        in.id = request.parameters.at(i).id;
        in.datatype = request.parameters.at(i).datatype;
        in.signature = request.parameters.at(i).signature;
        in.value = request.parameters.at(i).value;  // only needed when sending values
        instructions.instructions.push_back(in);
    }

    publisher_instructionsRequest.publish(instructions);
    lastSendTime_.restart();
    markBlockRequestSent(request.ip, request.parameters);
    InFlightRequest inFlightRequest;
    inFlightRequest.request = request;
    inFlightRequest.since.restart();
    inFlightRequests_[request.ip] = inFlightRequest;
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

        if (in.at(i).signature != parameter.at(i).signature)
            return false;
    }

    return true;
}

bool Node::compareParameterBlocks(const Node::SensorParameters &lhs, const Node::SensorParameters &rhs) const
{
    if (lhs.size() != rhs.size())
        return false;

    for (int i = 0; i < lhs.size(); i += 1)
    {
        if (lhs.at(i).request != rhs.at(i).request)
            return false;
        if (lhs.at(i).section != rhs.at(i).section)
            return false;
        if (lhs.at(i).id != rhs.at(i).id)
            return false;
        if (lhs.at(i).datatype != rhs.at(i).datatype)
            return false;
        if (lhs.at(i).signature != rhs.at(i).signature)
            return false;
    }

    return true;
}

void Node::writeBlock(const std::vector<smartmicro_driver::Instruction> &in, Node::SensorParameters &parameter)
{
    // assumes compareBlocks has already been checked

    for (int i = 0; i < in.size(); i += 1)
    {
        parameter[i].value = in.at(i).value;
        parameter[i].last_response = in.at(i).response;
    }
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
    blockCounter = (blockCounter + 1) % 6;  // sensors only read first instruction packet of a burst... so enforce delays

    for (int i = 0; i < sensors.count(); i += 1)
    {
        auto& s = sensors[i];
        bool const drvegrd = isDrvegrd(s.radar_type);

        if (!drvegrd && (blockCounter == 0) && ((!s.block2012.last_request.isValid()) || (s.block2012.last_request.elapsed() > 1000)))
            enqueuePollingRequest(s.ip, s.block2012.parameters);

        if (drvegrd && (blockCounter == 0) && ((!s.block7012.last_request.isValid()) || (s.block7012.last_request.elapsed() > 1000)))
            enqueuePollingRequest(s.ip, s.block7012.parameters);

        if (!drvegrd && (blockCounter == 1) && ((!s.block2010_meas.last_request.isValid()) || (s.block2010_meas.last_request.elapsed() > 1000)))
            enqueuePollingRequest(s.ip, s.block2010_meas.parameters);

        if (drvegrd && (blockCounter == 1) && ((!s.block7010_meas.last_request.isValid()) || (s.block7010_meas.last_request.elapsed() > 1000)))
            enqueuePollingRequest(s.ip, s.block7010_meas.parameters);

        if (!drvegrd && (blockCounter == 2) && ((!s.block2010_meas_132.last_request.isValid()) || (s.block2010_meas_132.last_request.elapsed() > 1000)))
            enqueuePollingRequest(s.ip, s.block2010_meas_132.parameters);

        if (drvegrd && (blockCounter == 2) && ((!s.block7010_modes.last_request.isValid()) || (s.block7010_modes.last_request.elapsed() > 1000)))
            enqueuePollingRequest(s.ip, s.block7010_modes.parameters);

        if (!drvegrd && (blockCounter == 3) && ((!s.block2010_meas_153.last_request.isValid()) || (s.block2010_meas_153.last_request.elapsed() > 1000)))
            enqueuePollingRequest(s.ip, s.block2010_meas_153.parameters);

        if (!drvegrd && (blockCounter == 4) && ((!s.block2010_output.last_request.isValid()) || (s.block2010_output.last_request.elapsed() > 1000)))
            enqueuePollingRequest(s.ip, s.block2010_output.parameters);

        if (drvegrd && (blockCounter == 4) && ((!s.block7010_output.last_request.isValid()) || (s.block7010_output.last_request.elapsed() > 1000)))
            enqueuePollingRequest(s.ip, s.block7010_output.parameters);

        if (drvegrd && (blockCounter == 5) && ((!s.block7010_ip.last_request.isValid()) || (s.block7010_ip.last_request.elapsed() > 1000)))
            enqueuePollingRequest(s.ip, s.block7010_ip.parameters);
    }

    processInstructionQueue();

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

            for (int j = 0; j <= 25; j += 1)
                ui->tableWidget->setItem(j, i, new QTableWidgetItem());
            ui->tableWidget->item(0,i)->setText(QString::number(s.ip).prepend("192.168.11."));
        }
    }

    // update tableWidget items
    for (int i = 0; i < sensors.count(); i += 1)
    {
        auto& s = sensors.at(i);

        QString sensor_ip = QString::number(s.ip).prepend("192.168.11.");
        QString sensor_type = "Unknown";
        if (s.radar_type == smartmicro_driver::Sensor::RADAR_TYPE_DRVEGRD)
            sensor_type = "DRVEGRD";
        else if (s.radar_type == smartmicro_driver::Sensor::RADAR_TYPE_UMRR)
            sensor_type = "UMRR";
        ui->tableWidget->item(0,i)->setText(sensor_ip);
        ui->tableWidget->item(RADAR_TYPE_DISPLAY,i)->setText(sensor_type);

        if (isDrvegrd(s.radar_type))
        {
            updateGuiBlock(i, s.block7012.parameters, s.block7012.last_response);
            updateGuiBlock(i, s.block7010_meas.parameters, s.block7010_meas.last_response);
            updateGuiBlock(i, s.block7010_modes.parameters, s.block7010_modes.last_response);
            updateGuiBlock(i, s.block7010_output.parameters, s.block7010_output.last_response);
            updateGuiBlock(i, s.block7010_ip.parameters, s.block7010_ip.last_response);
        }
        else
        {
            updateGuiBlock(i, s.block2012.parameters, s.block2012.last_response);
            updateGuiBlock(i, s.block2010_meas.parameters, s.block2010_meas.last_response);
            updateGuiBlock(i, s.block2010_meas_132.parameters, s.block2010_meas_132.last_response);
            updateGuiBlock(i, s.block2010_meas_153.parameters, s.block2010_meas_153.last_response);
            updateGuiBlock(i, s.block2010_output.parameters, s.block2010_output.last_response);
        }
    }
}

void Node::refreshControlAvailability()
{
    int idx = getSensorIdx(ui->destination->value());
    bool const drvegrd = (idx >= 0) && isDrvegrd(sensors.at(idx).radar_type);
    QString sensor_type = "Unknown";
    if (idx >= 0)
        sensor_type = drvegrd ? "DRVEGRD" : "UMRR";

    ui->frame_umrr->setVisible(!drvegrd);
    ui->frame_drvegrd->setVisible(drvegrd);
    ui->groupBox->setTitle("Configuration");
    ui->label_selectedTypeValue->setText(sensor_type);
}

void Node::updateGuiBlock(int sensor, const Node::SensorParameters &parameters, const QElapsedTimer &timer) const
{
    QBrush br;
    if (!timer.isValid())
        br = QBrush(QColor(100,100,100));
    else if (timer.elapsed() > staleDisplayTimeoutMs_)
        br =  QBrush(QColor("orange"));
    else
        br = clearBrushOfTableWidget;

    for (int j = 0; j < parameters.count(); j += 1)
    {
        auto* item = ui->tableWidget->item(parameters.at(j).display,sensor);
        if (!timer.isValid())
        {
            item->setText("");
            item->setBackground(br);
            item->setForeground(clearTextBrushOfTableWidget);
            continue;
        }

        if (parameters.at(j).last_response == smartmicro_driver::Instruction::RESPONSE_SUCCESS)
        {
            item->setText(parameterValueToString(parameters.at(j)));
            item->setBackground(br);
            item->setForeground(clearTextBrushOfTableWidget);
        }
        else if (parameters.at(j).last_response != smartmicro_driver::Instruction::RESPONSE_UNSET)
        {
            item->setText("error");
            item->setBackground(br);
            item->setForeground(QBrush(QColor("red")));
        }
        else
        {
            item->setText("");
            item->setBackground(br);
            item->setForeground(clearTextBrushOfTableWidget);
        }
    }
}

void Node::on_btn_txAntennaIdx_0_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 0, smartmicro_driver::Instruction::DATATYPE_U8, 0, 0, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_txAntennaIdx_1_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 0, smartmicro_driver::Instruction::DATATYPE_U8, 0, 1, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_txAntennaIdx_2_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 0, smartmicro_driver::Instruction::DATATYPE_U8, 0, 2, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_rangeToggleMode_off_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 5, smartmicro_driver::Instruction::DATATYPE_U8, 0, 0, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_rangeToggleMode_sm_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 5, smartmicro_driver::Instruction::DATATYPE_U8, 0, 1, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_rangeToggleMode_sl_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 5, smartmicro_driver::Instruction::DATATYPE_U8, 0, 2, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_rangeToggleMode_ml_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 5, smartmicro_driver::Instruction::DATATYPE_U8, 0, 3, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_txRangeToggle_on_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 3, smartmicro_driver::Instruction::DATATYPE_U8, 0, 1, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_txRangeToggle_off_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 3, smartmicro_driver::Instruction::DATATYPE_U8, 0, 0, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_angularSeparation_on_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 4, smartmicro_driver::Instruction::DATATYPE_U8, 0, 1, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_angularSeparation_off_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 4, smartmicro_driver::Instruction::DATATYPE_U8, 0, 0, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_frequencySweep_short_clicked()
{
    int idx = getSensorIdx(ui->destination->value());
    bool const drvegrd = (idx >= 0) && isDrvegrd(sensors.at(idx).radar_type);
    quint16 const section = static_cast<quint16>(drvegrd ? 7010 : 2010);
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, section, 2, smartmicro_driver::Instruction::DATATYPE_U8, drvegrd ? DRVEGRD_SIGNATURE_PARAM_FREQUENCY_SWEEP : 0, 2, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_frequencySweep_medium_clicked()
{
    int idx = getSensorIdx(ui->destination->value());
    bool const drvegrd = (idx >= 0) && isDrvegrd(sensors.at(idx).radar_type);
    quint16 const section = static_cast<quint16>(drvegrd ? 7010 : 2010);
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, section, 2, smartmicro_driver::Instruction::DATATYPE_U8, drvegrd ? DRVEGRD_SIGNATURE_PARAM_FREQUENCY_SWEEP : 0, 1, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_frequencySweep_long_clicked()
{
    int idx = getSensorIdx(ui->destination->value());
    bool const drvegrd = (idx >= 0) && isDrvegrd(sensors.at(idx).radar_type);
    quint16 const section = static_cast<quint16>(drvegrd ? 7010 : 2010);
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, section, 2, smartmicro_driver::Instruction::DATATYPE_U8, drvegrd ? DRVEGRD_SIGNATURE_PARAM_FREQUENCY_SWEEP : 0, 0, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_destination_valueChanged(const QString &arg1)
{
    if (arg1.isEmpty())
        ui->destination->setStyleSheet("background-color: rgba(255,0,0,100);");
    else
        ui->destination->setStyleSheet("");

    int ip = ui->destination->value();
    ui->ip_value->setValue(ip);
    ui->ip_value_drvegrd->setValue(ip);
    refreshControlAvailability();
}

void Node::on_btn_outputCan_on_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 200, smartmicro_driver::Instruction::DATATYPE_U8, 0, 1, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_outputCan_off_clicked()
{
    // ID 201 and ID 204 should be set to 0, too. However, they are no longer reported in the current documentation
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 200, smartmicro_driver::Instruction::DATATYPE_U8, 0, 0, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_outputEth_on_clicked()
{
    int idx = getSensorIdx(ui->destination->value());
    bool const drvegrd = (idx >= 0) && isDrvegrd(sensors.at(idx).radar_type);
    quint16 const section = static_cast<quint16>(drvegrd ? 7010 : 2010);
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, section, 202, smartmicro_driver::Instruction::DATATYPE_U8, drvegrd ? DRVEGRD_SIGNATURE_PARAM_OUTPUT_ETH : 0, 1, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_outputEth_off_clicked()
{
    int idx = getSensorIdx(ui->destination->value());
    bool const drvegrd = (idx >= 0) && isDrvegrd(sensors.at(idx).radar_type);
    quint16 const section = static_cast<quint16>(drvegrd ? 7010 : 2010);
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, section, 202, smartmicro_driver::Instruction::DATATYPE_U8, drvegrd ? DRVEGRD_SIGNATURE_PARAM_OUTPUT_ETH : 0, 0, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_speedInterval_btn_clicked()
{
    SensorParameters pars;
    double value = ui->speedInterval_value->value();

    if (ui->speedInterval_negLong->isChecked())
        pars += SensorParameter{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 20, smartmicro_driver::Instruction::DATATYPE_F32, 0, value, 0};
    if (ui->speedInterval_negMedium->isChecked())
        pars += SensorParameter{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 21, smartmicro_driver::Instruction::DATATYPE_F32, 0, value, 0};
    if (ui->speedInterval_negShort->isChecked())
        pars += SensorParameter{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 22, smartmicro_driver::Instruction::DATATYPE_F32, 0, value, 0};
    if (ui->speedInterval_posLong->isChecked())
        pars += SensorParameter{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 30, smartmicro_driver::Instruction::DATATYPE_F32, 0, value, 0};
    if (ui->speedInterval_posMedium->isChecked())
        pars += SensorParameter{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 31, smartmicro_driver::Instruction::DATATYPE_F32, 0, value, 0};
    if (ui->speedInterval_posShort->isChecked())
        pars += SensorParameter{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 2010, 32, smartmicro_driver::Instruction::DATATYPE_F32, 0, value, 0};

    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_ip_btn_clicked()
{
    int idx = getSensorIdx(ui->destination->value());
    bool const drvegrd = (idx >= 0) && isDrvegrd(sensors.at(idx).radar_type);
    quint32 ip_raw = 3232238353 - 17 + static_cast<quint32>(ui->ip_value->value());
    double ip_m = ip_raw;   // double mantisse large enough for uint32
    quint16 const section = static_cast<quint16>(drvegrd ? 7010 : 2010);
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, section, 231, smartmicro_driver::Instruction::DATATYPE_U32, drvegrd ? DRVEGRD_SIGNATURE_PARAM_IP_SOURCE : 0, ip_m, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_flash_btn_clicked()
{
    int idx = getSensorIdx(ui->destination->value());
    bool const drvegrd = (idx >= 0) && isDrvegrd(sensors.at(idx).radar_type);
    SensorParameters pars;
    if (drvegrd)
        pars += SensorParameter{smartmicro_driver::Instruction::REQUEST_COMMAND, 7000, 1, smartmicro_driver::Instruction::DATATYPE_U8, DRVEGRD_SIGNATURE_COMMAND_REBOOT, 0, 0};
    else
        pars += SensorParameter{smartmicro_driver::Instruction::REQUEST_COMMAND, 1000, 344, smartmicro_driver::Instruction::DATATYPE_U32, 0, 2010, 0};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_centerFrequency_0_drvegrd_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 7010, 1, smartmicro_driver::Instruction::DATATYPE_U8, DRVEGRD_SIGNATURE_PARAM_CENTER_FREQUENCY, 0, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_centerFrequency_1_drvegrd_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 7010, 1, smartmicro_driver::Instruction::DATATYPE_U8, DRVEGRD_SIGNATURE_PARAM_CENTER_FREQUENCY, 1, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_centerFrequency_2_drvegrd_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 7010, 1, smartmicro_driver::Instruction::DATATYPE_U8, DRVEGRD_SIGNATURE_PARAM_CENTER_FREQUENCY, 2, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_centerFrequency_3_drvegrd_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 7010, 1, smartmicro_driver::Instruction::DATATYPE_U8, DRVEGRD_SIGNATURE_PARAM_CENTER_FREQUENCY, 3, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_frequencySweep_short_drvegrd_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 7010, 2, smartmicro_driver::Instruction::DATATYPE_U8, DRVEGRD_SIGNATURE_PARAM_FREQUENCY_SWEEP, 2, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_frequencySweep_medium_drvegrd_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 7010, 2, smartmicro_driver::Instruction::DATATYPE_U8, DRVEGRD_SIGNATURE_PARAM_FREQUENCY_SWEEP, 1, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_frequencySweep_long_drvegrd_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 7010, 2, smartmicro_driver::Instruction::DATATYPE_U8, DRVEGRD_SIGNATURE_PARAM_FREQUENCY_SWEEP, 0, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_hires_on_drvegrd_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 7010, 10, smartmicro_driver::Instruction::DATATYPE_U8, DRVEGRD_SIGNATURE_PARAM_HIRES_MODE, 1, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_hires_off_drvegrd_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 7010, 10, smartmicro_driver::Instruction::DATATYPE_U8, DRVEGRD_SIGNATURE_PARAM_HIRES_MODE, 0, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_targetValidation_on_drvegrd_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 7010, 11, smartmicro_driver::Instruction::DATATYPE_U8, DRVEGRD_SIGNATURE_PARAM_TARGET_VALIDATION, 1, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_targetValidation_off_drvegrd_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 7010, 11, smartmicro_driver::Instruction::DATATYPE_U8, DRVEGRD_SIGNATURE_PARAM_TARGET_VALIDATION, 0, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_outputEth_on_drvegrd_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 7010, 202, smartmicro_driver::Instruction::DATATYPE_U8, DRVEGRD_SIGNATURE_PARAM_OUTPUT_ETH, 1, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_btn_outputEth_off_drvegrd_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 7010, 202, smartmicro_driver::Instruction::DATATYPE_U8, DRVEGRD_SIGNATURE_PARAM_OUTPUT_ETH, 0, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_ip_btn_drvegrd_clicked()
{
    quint32 ip_raw = 3232238353 - 17 + static_cast<quint32>(ui->ip_value_drvegrd->value());
    double ip_m = ip_raw;
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_PARAMETER_SET, 7010, 231, smartmicro_driver::Instruction::DATATYPE_U32, DRVEGRD_SIGNATURE_PARAM_IP_SOURCE, ip_m, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_flash_btn_drvegrd_clicked()
{
    SensorParameters pars{{smartmicro_driver::Instruction::REQUEST_COMMAND, 7000, 1, smartmicro_driver::Instruction::DATATYPE_U8, DRVEGRD_SIGNATURE_COMMAND_REBOOT, 0, 0}};
    requestInstructions(static_cast<quint8>(ui->destination->value()), pars);
}

void Node::on_tableWidget_cellClicked(int, int column)
{
    if (column < 0 || column >= sensors.count())
        return;

    ui->destination->setValue(sensors.at(column).ip);
    refreshControlAvailability();
}
