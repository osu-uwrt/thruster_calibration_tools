#include <ui_thruster_cal_gui.h>
#include <QApplication>
#include <QMainWindow>
#include <QFileDialog>
#include <QTimer>
#include <QPushButton>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <riptide_msgs2/msg/dshot_command.hpp>
#include <riptide_msgs2/msg/dshot_rpm_feedback.hpp>
#include <riptide_msgs2/msg/dshot_partial_telemetry.hpp>
#include <thruster_cal_msgs/action/calibrate_thruster.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

#define MSG_STALE_TIME 1s
#define ACTION_NAME "calibrate_thruster"

using namespace std::placeholders;
using namespace std::chrono_literals;

class ThrusterCalGuiNode : public rclcpp::Node
{
    public:
    typedef std::shared_ptr<ThrusterCalGuiNode> SharedPtr;
    using CalibrateThruster = thruster_cal_msgs::action::CalibrateThruster;
    using CalibrateThrusterGH = rclcpp_action::ClientGoalHandle<CalibrateThruster>;

    ThrusterCalGuiNode(Ui_ThrusterCalGui *ui)
     : rclcpp::Node("thruster_cal_gui"),
       ui(ui),
       doRecurrentManualPub(false)
    {
        // connect buttons
        ui->centralwidget->connect(ui->sendManualCommand, &QPushButton::clicked, [this]() { this->publishDShotButtonPressed(); });
        ui->centralwidget->connect(ui->setRawButton, &QPushButton::clicked, [this]() { this->publishManualSetRawState(); });
        ui->centralwidget->connect(ui->publishTrigger, &QPushButton::clicked, [this]() { this->publishTrigger(); });
        ui->centralwidget->connect(ui->calibFileBrowse, &QPushButton::clicked, [this]() { this->browseLogfile(); });
        ui->centralwidget->connect(ui->procedureButton, &QPushButton::clicked, [this]() { this->procedureButtonClicked(); });
        ui->centralwidget->connect(ui->resetGauge, &QPushButton::clicked, [this]() { this->resetForceGauge(); });

        // initialize timer
        this->timer = create_wall_timer(50ms, std::bind(&ThrusterCalGuiNode::timerCb, this));

        // initialize action client
        this->calibrationClient = rclcpp_action::create_client<CalibrateThruster>(
            this, ACTION_NAME);

        //initialize pubs
        this->commandPub = create_publisher<riptide_msgs2::msg::DshotCommand>(
            "command/thruster_rpm", rclcpp::SensorDataQoS());
        
        this->triggerPub = create_publisher<std_msgs::msg::Empty>(
            "command/trigger", 10);
        
        this->setRawPub = create_publisher<std_msgs::msg::Bool>(
            "dshot_tune/set_raw", 10);

        this->forceGaugeShutdownPub = create_publisher<std_msgs::msg::Empty>(
            "force_gauge/shutdown", 10);

        // initialize subs
        this->thrusterTelemetrySub = create_subscription<riptide_msgs2::msg::DshotPartialTelemetry>(
            "state/thrusters",
            rclcpp::SensorDataQoS(),
            std::bind(&ThrusterCalGuiNode::thrusterTelemetryCb, this, _1));
        
        this->dshotCommandSub = create_subscription<riptide_msgs2::msg::DshotCommand>(
            "command/thruster_rpm",
            rclcpp::SensorDataQoS(),
            std::bind(&ThrusterCalGuiNode::dshotCommandCb, this, _1));
        
        this->rpmSub = create_subscription<riptide_msgs2::msg::DshotRPMFeedback>(
            "state/thrusters/rpm_complete",
            rclcpp::SensorDataQoS(),
            std::bind(&ThrusterCalGuiNode::rpmCb, this, _1));
        
        this->forceSub = create_subscription<std_msgs::msg::Float32>(
            "force_gauge/force",
            rclcpp::SensorDataQoS(),
            std::bind(&ThrusterCalGuiNode::forceCb, this, _1));
        
        this->calibratingSub = create_subscription<std_msgs::msg::Bool>(
            "state/thruster_cal/calibrating",
            10,
            std::bind(&ThrusterCalGuiNode::calibratingCb, this, _1));
        
        this->rawStateSub = create_subscription<std_msgs::msg::Bool>(
            "dshot_tune/set_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&ThrusterCalGuiNode::rawStateCb, this, _1));

        // initialize state
        rclcpp::Time now = get_clock()->now();
        lastHardwareMsgTime = now;
        lastForceGaugeMsgTime = now;
        lastCalNodeMsgTime = now;
        lastRpmEchoMsgTime = now;
    }
    
    private:
    
    void setStatus(const std::string& status, bool error)
    {
        if(error)
        {
            ui->calibStatus->setStyleSheet("color: red");
            RCLCPP_ERROR(get_logger(), status.c_str());
        } else
        {
            ui->calibStatus->setStyleSheet("color: black");
            RCLCPP_INFO(get_logger(), status.c_str());
        }

        ui->calibStatus->setText(QString::fromStdString(status));
    }

    void timerCb()
    {
        rclcpp::Time now = get_clock()->now();

        // update ROS time indicator
        ui->rosTime->setText(
            QString::fromStdString(std::to_string(get_clock()->now().seconds())));
        
        // update online indicators
        ui->thrusterOnline->setChecked(now - lastHardwareMsgTime < MSG_STALE_TIME);
        ui->forceGaugeOnline->setChecked(now - lastForceGaugeMsgTime < MSG_STALE_TIME);
        ui->calibNodeOnline->setChecked(now - lastCalNodeMsgTime < MSG_STALE_TIME);
        ui->rpmEchoNodeOnline->setChecked(now - lastRpmEchoMsgTime < MSG_STALE_TIME);

        if(doRecurrentManualPub)
        {
            publishManualDShotCommandOnTimer();
        }
    }


    void publishDShotButtonPressed()
    {
        doRecurrentManualPub = !doRecurrentManualPub;
    }


    void publishManualDShotCommandOnTimer()
    {
        riptide_msgs2::msg::DshotCommand cmd;
        int 
            targetThruster = ui->targetThruster->value(),
            dshotValue = ui->manualDshotCommand->value();

        if(targetThruster < (int) cmd.values.size())
        {
            cmd.values[targetThruster] = dshotValue;
            commandPub->publish(cmd);
        }
    }
    

    void publishManualSetRawState()
    {
        std_msgs::msg::Bool msg;
        msg.data = ui->setRaw->isChecked();
        setRawPub->publish(msg);
    }


    void publishTrigger()
    {
        std_msgs::msg::Empty empty;
        this->triggerPub->publish(empty);
    }


    void resetForceGauge()
    {
        std_msgs::msg::Empty msg;
        forceGaugeShutdownPub->publish(msg);
    }


    void browseLogfile()
    {
        QString dirPath = QFileDialog::getSaveFileName(ui->centralwidget, "Browse for log file", "~");
        if(!dirPath.endsWith(".csv"))
        {
            dirPath += ".csv";
        }

        ui->calibFile->setText(dirPath);
    }


    void procedureButtonClicked()
    {
        if(ui->calibrating->isChecked())
        {
            setStatus("Canceling goal", false);
            calibrationClient->async_cancel_all_goals();
            return;
        }

        setStatus("Setting raw mode into firmware", false);
        std_msgs::msg::Bool setRawMsg;
        setRawMsg.data = true;
        this->setRawPub->publish(setRawMsg);

        setStatus("Attempting to begin procedure", false);

        if(!calibrationClient->wait_for_action_server(1s))
        {
            setStatus("Timed out waiting for action server on " ACTION_NAME ".", true);
            return;
        }

        setStatus("Action server at " ACTION_NAME " is available, sending goal...", false);

        auto goal = CalibrateThruster::Goal();
        goal.thruster_num = ui->targetThruster->value();
        goal.dshot_min = ui->calibDShotMin->value();
        goal.dshot_max = ui->calibDShotMax->value();
        goal.step_size = ui->calibDShotStep->value();
        goal.file = ui->calibFile->text().toStdString();

        auto goalOptions = rclcpp_action::Client<CalibrateThruster>::SendGoalOptions();
        
        goalOptions.goal_response_callback =
            std::bind(&ThrusterCalGuiNode::goalResponseCb, this, _1);
        goalOptions.feedback_callback =
            std::bind(&ThrusterCalGuiNode::feedbackCb, this, _1, _2);
        goalOptions.result_callback =
            std::bind(&ThrusterCalGuiNode::resultCb, this, _1);
        
        this->calibrationClient->async_send_goal(goal, goalOptions);

    }


    void goalResponseCb(CalibrateThrusterGH::SharedPtr handle)
    {
        if (!handle) 
        {
            setStatus("Goal was rejected by server", true);
        } else 
        {
            setStatus("Goal accepted by server. Procedure in progress", true);
        }
    }


    void feedbackCb(
        CalibrateThrusterGH::SharedPtr,
        const std::shared_ptr<const CalibrateThruster::Feedback> feedback)
    {
        ui->calibCurrentIndex->setText(QString::fromStdString(std::to_string(feedback->current_dshot)));
        setStatus(feedback->message, false);
    }


    void resultCb(const CalibrateThrusterGH::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                setStatus("Procedure succeeded!", false);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                setStatus("Goal was aborted", true);
                break;
            case rclcpp_action::ResultCode::CANCELED:
                setStatus("Goal was canceled", true);
                break;
            default:
                setStatus("Unknown result code", true);
                break;
        }
    }


    void thrusterTelemetryCb(riptide_msgs2::msg::DshotPartialTelemetry::ConstSharedPtr)
    {
        lastHardwareMsgTime = get_clock()->now();
    }


    void rpmCb(riptide_msgs2::msg::DshotRPMFeedback::ConstSharedPtr msg)
    {
        lastRpmEchoMsgTime = get_clock()->now();
        int targetThruster = ui->targetThruster->value();
        if(targetThruster < (int) msg->rpm.size())
        {
            // if rpm is valid...
            if(msg->rpm_valid_mask & (1 << targetThruster) & 0xFF)
            {
                ui->currentRPM->setText(QString::fromStdString(std::to_string(msg->rpm[targetThruster])));
            } else
            {
                ui->currentRPM->setText("Invalid");
            }
        }
    }


    void dshotCommandCb(riptide_msgs2::msg::DshotCommand::ConstSharedPtr msg)
    {
        int targetThruster = ui->targetThruster->value();
        if(targetThruster < (int) msg->values.size())
        {
            ui->currentDShot->setText(QString::fromStdString(std::to_string(msg->values[targetThruster])));
        }
    }


    void forceCb(std_msgs::msg::Float32::ConstSharedPtr msg)
    {
        lastForceGaugeMsgTime = get_clock()->now();
        ui->currentForce->setText(QString::fromStdString(std::to_string(msg->data)));
    }


    void calibratingCb(std_msgs::msg::Bool::ConstSharedPtr msg)
    {
        lastCalNodeMsgTime = get_clock()->now();
        ui->calibrating->setChecked(msg->data);
        
        //set procedure button label
        ui->procedureButton->setText(msg->data ? "Cancel Procedure" : "Begin Procedure");
    }


    void rawStateCb(std_msgs::msg::Bool::ConstSharedPtr msg)
    {
        ui->rawState->setChecked(msg->data);
    }


    Ui_ThrusterCalGui *ui;
    rclcpp::TimerBase::SharedPtr timer;

    bool doRecurrentManualPub;

    // msg timestamps
    rclcpp::Time
        lastHardwareMsgTime,
        lastForceGaugeMsgTime,
        lastCalNodeMsgTime,
        lastRpmEchoMsgTime;
    
    // action client
    rclcpp_action::Client<CalibrateThruster>::SharedPtr calibrationClient;

    // msg publishers
    rclcpp::Publisher<riptide_msgs2::msg::DshotCommand>::SharedPtr commandPub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr
        triggerPub,
        forceGaugeShutdownPub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr setRawPub;

    // msg subscriptions
    rclcpp::Subscription<riptide_msgs2::msg::DshotPartialTelemetry>::SharedPtr thrusterTelemetrySub;
    rclcpp::Subscription<riptide_msgs2::msg::DshotCommand>::SharedPtr dshotCommandSub;
    rclcpp::Subscription<riptide_msgs2::msg::DshotRPMFeedback>::SharedPtr rpmSub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr forceSub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr 
        calibratingSub,
        rawStateSub;
};

ThrusterCalGuiNode::SharedPtr node;

void rosMain(Ui_ThrusterCalGui *ui)
{
    node = std::make_shared<ThrusterCalGuiNode>(ui);
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QMainWindow w;
    Ui_ThrusterCalGui *ui = new Ui_ThrusterCalGui();
    ui->setupUi(&w);
    rclcpp::init(argc, argv);
    rosMain(ui);
    QTimer timer(ui->centralwidget);
    timer.callOnTimeout([](){ rclcpp::spin_some(node); });
    timer.setInterval(1ms);
    timer.start();
    w.show();
    int guiResult = a.exec();
    rclcpp::shutdown();
    return guiResult;
}
