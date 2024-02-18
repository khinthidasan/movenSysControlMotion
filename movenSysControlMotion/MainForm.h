#pragma once

//Library from MovenSys WOS
#include <WMX3Api.h>
#include <WMX3Funcs.h>
#include <CoreMotionApi.h>
#include <EcApi.h>
#include <IOApi.h>

//C++ Library
#include <iostream>
#include <string>
#include <msclr/marshal_cppstd.h>
#include <thread>
#include <sstream>
#include <iomanip>


/*****************************************************************************/
/* Define                                                                    */
/*****************************************************************************/
#define OP_STATE(state) \
    ((state == OperationState::Idle)          ? "Idle"          : \
     (state == OperationState::Pos)           ? "Pos"           : \
     (state == OperationState::Jog)           ? "Jog"           : \
     (state == OperationState::Home)          ? "Home"          : \
     (state == OperationState::Sync)          ? "Sync"          : \
     (state == OperationState::GantryHome)    ? "GantryHome"    : \
     (state == OperationState::Stop)          ? "Stop"          : \
     (state == OperationState::Intpl)         ? "Intpl"         : \
     (state == OperationState::ConstLinearVelocity) ? \
                                    "ConstLinearVelocity"       : \
     (state == OperationState::Trq)           ? "Trq"           : \
     (state == OperationState::DirectControl)       ? \
                                    "DirectControl"             : \
     (state == OperationState::PVT)           ? "PVT"           : \
     (state == OperationState::ECAM)          ? "ECAM"          : \
     (state == OperationState::SyncCatchUp)   ? "SyncCatchUp"   : \
     (state == OperationState::DancerControl) ? "DancerControl" : \
     "")


namespace movenSysControlMotion {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	//namespace releated to MovenSys WOS
	using namespace wmx3Api;
	using namespace std;
	using namespace ecApi;

	//variable releated to MovenSys WOS
	WMX3Api Device;
	CoreMotionStatus CmStatus;
	CoreMotionAxisStatus* cmAxis;

	Motion::PosCommand pos;
	CoreMotion wmxlib_cm(&Device);

	EngineStatus engStatus;
	OperationState::T servoState;
	Io  Wmx3Lib_Io(&Device);

	//defined variable 
	char defaultDeviceName[] = "Lab2mDevice";
	char pathDir[] = "C:\\Program Files\\SoftServo\\WMX3\\";
	bool communicationOnOff = false;
	bool servoOnOff0 = false;
	int err;
	char errString[256];
	int sensorDataInterval = 1;  //for PGV and motor data (data munipulation timer to show on the from)

	/// COM serial variable
	HANDLE hSerial;
	DCB dcbSerialParams = { 0 };
	COMMTIMEOUTS timeouts = { 0 };
	DWORD bytesWritten;
	DWORD bytesRead;
	char dataToRequest[] = "\xC8\x37\xFF";
	char dataToRequestRightLane[] = "\xE4\x1B\xFF";
	char dataToRequestLeftLane[] = "\xC8\xE8\x17";
	char receivedData[22]; // 21 characters + null terminator
	char receivedDataShown[22]; // 21 characters + null terminator 
	bool portState = false;  //COM port state for PGV connection
	bool PGVStart = false; //PGV request start & Stop communication
	bool isAGVTagPosition = false;


	//define speed & velocity for NORMAL Operation
	double normalJogSpeed = 5000;
	double normalAccelSpeed = 5000;
	double normalDecelSpeed = 5000;

	//define speed & velocity for SLOW Operation
	double slowJogSpeed = 1000;
	double slowAccelSpeed = 1000;
	double slowDecelSpeed = 1000;

	/// <summary>
	/// Summary for MainForm
	/// </summary>
	public ref class MainForm : public System::Windows::Forms::Form
	{
	public:
		MainForm(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~MainForm()
		{
			if (components)
			{
				delete components;
			}
		}

	protected:

	//define thread
	private: static System::Threading::Thread^ thrPGVDataReceiving; //PGV sensor data reading
	private: static System::Threading::Thread^ thrAwaitSlow; //Slow By QR Code
	private: static System::Threading::Thread^ thrAwaitStop; //Stop By IO sensor

	private: System::Windows::Forms::RichTextBox^ richTextBoxMessage;
	private: System::Windows::Forms::Button^ button6;
	private: System::Windows::Forms::Button^ button1;
	private: System::Windows::Forms::Button^ button_create_device;
	private: System::Windows::Forms::Button^ button_communication;
	private: System::Windows::Forms::Button^ button_clear_message;
	private: System::Windows::Forms::Button^ button_open_port;
	private: System::Windows::Forms::Button^ button_connect_PGV;
	private: System::Windows::Forms::Button^ button_servo;
	private: System::Windows::Forms::GroupBox^ groupBox4;
	private: System::Windows::Forms::Label^ label49;
	private: System::Windows::Forms::Label^ labelStatusActVel3;
	private: System::Windows::Forms::Label^ labelStatusActVel2;
	private: System::Windows::Forms::Label^ labelStatusActVel0;
	private: System::Windows::Forms::Label^ labelStatusVelCmd3;
	private: System::Windows::Forms::Label^ labelStatusVelCmd2;
	private: System::Windows::Forms::Label^ labelStatusVelCmd0;
	private: System::Windows::Forms::Label^ labelStatusActPos0;
	private: System::Windows::Forms::Label^ labelStatusPosCmd0;
	private: System::Windows::Forms::Label^ labelStatusOP0;
	private: System::Windows::Forms::Label^ label8;
	private: System::Windows::Forms::Label^ label7;
	private: System::Windows::Forms::Label^ label6;
	private: System::Windows::Forms::Label^ label5;
	private: System::Windows::Forms::Label^ labelStatusAxis0;
	private: System::Windows::Forms::GroupBox^ groupBox14;
	private: System::Windows::Forms::Label^ label_agv_angleValueDecimal;
	private: System::Windows::Forms::Label^ label_agv_warningValue;
	private: System::Windows::Forms::Label^ label_agv_yValue;
	private: System::Windows::Forms::Label^ label_agv_xValue;
	private: System::Windows::Forms::Label^ label_agv_angleValue;
	private: System::Windows::Forms::Label^ label_agv_tagValue;
	private: System::Windows::Forms::Label^ label53;
	private: System::Windows::Forms::Label^ label52;
	private: System::Windows::Forms::Label^ label51;
	private: System::Windows::Forms::Label^ label50;
	private: System::Windows::Forms::Label^ label47;
	private: System::Windows::Forms::Button^ button_start_motion;
	private: System::Windows::Forms::TextBox^ textBoxDeviceName;
	private: System::Windows::Forms::Button^ button_stop_motion;
	private: System::Windows::Forms::GroupBox^ groupBox1;
	private: System::Windows::Forms::GroupBox^ groupBox2;

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>
		System::ComponentModel::Container ^components;

#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->richTextBoxMessage = (gcnew System::Windows::Forms::RichTextBox());
			this->button6 = (gcnew System::Windows::Forms::Button());
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->button_create_device = (gcnew System::Windows::Forms::Button());
			this->button_communication = (gcnew System::Windows::Forms::Button());
			this->button_clear_message = (gcnew System::Windows::Forms::Button());
			this->button_open_port = (gcnew System::Windows::Forms::Button());
			this->button_connect_PGV = (gcnew System::Windows::Forms::Button());
			this->button_servo = (gcnew System::Windows::Forms::Button());
			this->groupBox4 = (gcnew System::Windows::Forms::GroupBox());
			this->label49 = (gcnew System::Windows::Forms::Label());
			this->labelStatusActVel3 = (gcnew System::Windows::Forms::Label());
			this->labelStatusActVel2 = (gcnew System::Windows::Forms::Label());
			this->labelStatusActVel0 = (gcnew System::Windows::Forms::Label());
			this->labelStatusVelCmd3 = (gcnew System::Windows::Forms::Label());
			this->labelStatusVelCmd2 = (gcnew System::Windows::Forms::Label());
			this->labelStatusVelCmd0 = (gcnew System::Windows::Forms::Label());
			this->labelStatusActPos0 = (gcnew System::Windows::Forms::Label());
			this->labelStatusPosCmd0 = (gcnew System::Windows::Forms::Label());
			this->labelStatusOP0 = (gcnew System::Windows::Forms::Label());
			this->label8 = (gcnew System::Windows::Forms::Label());
			this->label7 = (gcnew System::Windows::Forms::Label());
			this->label6 = (gcnew System::Windows::Forms::Label());
			this->label5 = (gcnew System::Windows::Forms::Label());
			this->labelStatusAxis0 = (gcnew System::Windows::Forms::Label());
			this->groupBox14 = (gcnew System::Windows::Forms::GroupBox());
			this->label_agv_angleValueDecimal = (gcnew System::Windows::Forms::Label());
			this->label_agv_warningValue = (gcnew System::Windows::Forms::Label());
			this->label_agv_yValue = (gcnew System::Windows::Forms::Label());
			this->label_agv_xValue = (gcnew System::Windows::Forms::Label());
			this->label_agv_angleValue = (gcnew System::Windows::Forms::Label());
			this->label_agv_tagValue = (gcnew System::Windows::Forms::Label());
			this->label53 = (gcnew System::Windows::Forms::Label());
			this->label52 = (gcnew System::Windows::Forms::Label());
			this->label51 = (gcnew System::Windows::Forms::Label());
			this->label50 = (gcnew System::Windows::Forms::Label());
			this->label47 = (gcnew System::Windows::Forms::Label());
			this->button_start_motion = (gcnew System::Windows::Forms::Button());
			this->textBoxDeviceName = (gcnew System::Windows::Forms::TextBox());
			this->button_stop_motion = (gcnew System::Windows::Forms::Button());
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->groupBox2 = (gcnew System::Windows::Forms::GroupBox());
			this->groupBox4->SuspendLayout();
			this->groupBox14->SuspendLayout();
			this->groupBox1->SuspendLayout();
			this->groupBox2->SuspendLayout();
			this->SuspendLayout();
			// 
			// richTextBoxMessage
			// 
			this->richTextBoxMessage->Location = System::Drawing::Point(16, 274);
			this->richTextBoxMessage->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->richTextBoxMessage->Name = L"richTextBoxMessage";
			this->richTextBoxMessage->Size = System::Drawing::Size(530, 87);
			this->richTextBoxMessage->TabIndex = 9;
			this->richTextBoxMessage->Text = L"";
			// 
			// button6
			// 
			this->button6->Font = (gcnew System::Drawing::Font(L"Gulim", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(129)));
			this->button6->Location = System::Drawing::Point(-712, 153);
			this->button6->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->button6->Name = L"button6";
			this->button6->Size = System::Drawing::Size(400, 113);
			this->button6->TabIndex = 10;
			this->button6->Text = L"Create Device";
			this->button6->UseVisualStyleBackColor = true;
			// 
			// button1
			// 
			this->button1->Enabled = false;
			this->button1->Font = (gcnew System::Drawing::Font(L"Gulim", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(129)));
			this->button1->Location = System::Drawing::Point(-712, 192);
			this->button1->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(400, 113);
			this->button1->TabIndex = 8;
			this->button1->Text = L"Stopped";
			this->button1->UseVisualStyleBackColor = true;
			// 
			// button_create_device
			// 
			this->button_create_device->Font = (gcnew System::Drawing::Font(L"Gulim", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(129)));
			this->button_create_device->Location = System::Drawing::Point(22, 43);
			this->button_create_device->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->button_create_device->Name = L"button_create_device";
			this->button_create_device->Size = System::Drawing::Size(136, 59);
			this->button_create_device->TabIndex = 12;
			this->button_create_device->Text = L"Create Device";
			this->button_create_device->UseVisualStyleBackColor = true;
			this->button_create_device->Click += gcnew System::EventHandler(this, &MainForm::button_create_device_Click);
			// 
			// button_communication
			// 
			this->button_communication->Enabled = false;
			this->button_communication->Font = (gcnew System::Drawing::Font(L"Gulim", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(129)));
			this->button_communication->Location = System::Drawing::Point(164, 45);
			this->button_communication->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->button_communication->Name = L"button_communication";
			this->button_communication->Size = System::Drawing::Size(136, 59);
			this->button_communication->TabIndex = 11;
			this->button_communication->Text = L"Stopped";
			this->button_communication->UseVisualStyleBackColor = true;
			this->button_communication->Click += gcnew System::EventHandler(this, &MainForm::button_communication_Click);
			// 
			// button_clear_message
			// 
			this->button_clear_message->Font = (gcnew System::Drawing::Font(L"Gulim", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(129)));
			this->button_clear_message->Location = System::Drawing::Point(484, 365);
			this->button_clear_message->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->button_clear_message->Name = L"button_clear_message";
			this->button_clear_message->Size = System::Drawing::Size(62, 26);
			this->button_clear_message->TabIndex = 174;
			this->button_clear_message->Text = L"Clear Message";
			this->button_clear_message->UseVisualStyleBackColor = true;
			this->button_clear_message->Click += gcnew System::EventHandler(this, &MainForm::button_clear_message_Click);
			// 
			// button_open_port
			// 
			this->button_open_port->Font = (gcnew System::Drawing::Font(L"Gulim", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(129)));
			this->button_open_port->Location = System::Drawing::Point(19, 43);
			this->button_open_port->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->button_open_port->Name = L"button_open_port";
			this->button_open_port->Size = System::Drawing::Size(136, 59);
			this->button_open_port->TabIndex = 176;
			this->button_open_port->Text = L"Open Port";
			this->button_open_port->UseVisualStyleBackColor = true;
			this->button_open_port->Click += gcnew System::EventHandler(this, &MainForm::button_open_port_Click);
			// 
			// button_connect_PGV
			// 
			this->button_connect_PGV->Font = (gcnew System::Drawing::Font(L"Gulim", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(129)));
			this->button_connect_PGV->Location = System::Drawing::Point(161, 45);
			this->button_connect_PGV->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->button_connect_PGV->Name = L"button_connect_PGV";
			this->button_connect_PGV->Size = System::Drawing::Size(136, 59);
			this->button_connect_PGV->TabIndex = 175;
			this->button_connect_PGV->Text = L"PGV";
			this->button_connect_PGV->UseVisualStyleBackColor = true;
			this->button_connect_PGV->Click += gcnew System::EventHandler(this, &MainForm::button_connect_PGV_Click);
			// 
			// button_servo
			// 
			this->button_servo->Font = (gcnew System::Drawing::Font(L"Gulim", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(129)));
			this->button_servo->Location = System::Drawing::Point(306, 44);
			this->button_servo->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->button_servo->Name = L"button_servo";
			this->button_servo->Size = System::Drawing::Size(136, 59);
			this->button_servo->TabIndex = 177;
			this->button_servo->Text = L"Servo On";
			this->button_servo->UseVisualStyleBackColor = true;
			this->button_servo->Click += gcnew System::EventHandler(this, &MainForm::button_servo_Click);
			// 
			// groupBox4
			// 
			this->groupBox4->BackColor = System::Drawing::Color::Black;
			this->groupBox4->Controls->Add(this->label49);
			this->groupBox4->Controls->Add(this->labelStatusActVel3);
			this->groupBox4->Controls->Add(this->labelStatusActVel2);
			this->groupBox4->Controls->Add(this->labelStatusActVel0);
			this->groupBox4->Controls->Add(this->labelStatusVelCmd3);
			this->groupBox4->Controls->Add(this->labelStatusVelCmd2);
			this->groupBox4->Controls->Add(this->labelStatusVelCmd0);
			this->groupBox4->Controls->Add(this->labelStatusActPos0);
			this->groupBox4->Controls->Add(this->labelStatusPosCmd0);
			this->groupBox4->Controls->Add(this->labelStatusOP0);
			this->groupBox4->Controls->Add(this->label8);
			this->groupBox4->Controls->Add(this->label7);
			this->groupBox4->Controls->Add(this->label6);
			this->groupBox4->Controls->Add(this->label5);
			this->groupBox4->Controls->Add(this->labelStatusAxis0);
			this->groupBox4->ForeColor = System::Drawing::SystemColors::ControlLightLight;
			this->groupBox4->Location = System::Drawing::Point(19, 110);
			this->groupBox4->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->groupBox4->Name = L"groupBox4";
			this->groupBox4->Padding = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->groupBox4->Size = System::Drawing::Size(483, 134);
			this->groupBox4->TabIndex = 178;
			this->groupBox4->TabStop = false;
			// 
			// label49
			// 
			this->label49->AutoSize = true;
			this->label49->Font = (gcnew System::Drawing::Font(L"Gadugi", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
			this->label49->ForeColor = System::Drawing::Color::Yellow;
			this->label49->Location = System::Drawing::Point(7, 0);
			this->label49->Name = L"label49";
			this->label49->Size = System::Drawing::Size(90, 19);
			this->label49->TabIndex = 163;
			this->label49->Text = L"Motor Data";
			// 
			// labelStatusActVel3
			// 
			this->labelStatusActVel3->AutoSize = true;
			this->labelStatusActVel3->ForeColor = System::Drawing::Color::GreenYellow;
			this->labelStatusActVel3->Location = System::Drawing::Point(749, 166);
			this->labelStatusActVel3->Name = L"labelStatusActVel3";
			this->labelStatusActVel3->Size = System::Drawing::Size(15, 15);
			this->labelStatusActVel3->TabIndex = 28;
			this->labelStatusActVel3->Text = L"0";
			// 
			// labelStatusActVel2
			// 
			this->labelStatusActVel2->AutoSize = true;
			this->labelStatusActVel2->ForeColor = System::Drawing::Color::GreenYellow;
			this->labelStatusActVel2->Location = System::Drawing::Point(749, 140);
			this->labelStatusActVel2->Name = L"labelStatusActVel2";
			this->labelStatusActVel2->Size = System::Drawing::Size(15, 15);
			this->labelStatusActVel2->TabIndex = 27;
			this->labelStatusActVel2->Text = L"0";
			// 
			// labelStatusActVel0
			// 
			this->labelStatusActVel0->AutoSize = true;
			this->labelStatusActVel0->ForeColor = System::Drawing::Color::GreenYellow;
			this->labelStatusActVel0->Location = System::Drawing::Point(346, 82);
			this->labelStatusActVel0->Name = L"labelStatusActVel0";
			this->labelStatusActVel0->Size = System::Drawing::Size(15, 15);
			this->labelStatusActVel0->TabIndex = 25;
			this->labelStatusActVel0->Text = L"0";
			// 
			// labelStatusVelCmd3
			// 
			this->labelStatusVelCmd3->AutoSize = true;
			this->labelStatusVelCmd3->ForeColor = System::Drawing::Color::GreenYellow;
			this->labelStatusVelCmd3->Location = System::Drawing::Point(592, 166);
			this->labelStatusVelCmd3->Name = L"labelStatusVelCmd3";
			this->labelStatusVelCmd3->Size = System::Drawing::Size(15, 15);
			this->labelStatusVelCmd3->TabIndex = 24;
			this->labelStatusVelCmd3->Text = L"0";
			// 
			// labelStatusVelCmd2
			// 
			this->labelStatusVelCmd2->AutoSize = true;
			this->labelStatusVelCmd2->ForeColor = System::Drawing::Color::GreenYellow;
			this->labelStatusVelCmd2->Location = System::Drawing::Point(592, 140);
			this->labelStatusVelCmd2->Name = L"labelStatusVelCmd2";
			this->labelStatusVelCmd2->Size = System::Drawing::Size(15, 15);
			this->labelStatusVelCmd2->TabIndex = 23;
			this->labelStatusVelCmd2->Text = L"0";
			// 
			// labelStatusVelCmd0
			// 
			this->labelStatusVelCmd0->AutoSize = true;
			this->labelStatusVelCmd0->ForeColor = System::Drawing::Color::GreenYellow;
			this->labelStatusVelCmd0->Location = System::Drawing::Point(346, 54);
			this->labelStatusVelCmd0->Name = L"labelStatusVelCmd0";
			this->labelStatusVelCmd0->Size = System::Drawing::Size(15, 15);
			this->labelStatusVelCmd0->TabIndex = 21;
			this->labelStatusVelCmd0->Text = L"0";
			// 
			// labelStatusActPos0
			// 
			this->labelStatusActPos0->AutoSize = true;
			this->labelStatusActPos0->ForeColor = System::Drawing::Color::GreenYellow;
			this->labelStatusActPos0->Location = System::Drawing::Point(125, 82);
			this->labelStatusActPos0->Name = L"labelStatusActPos0";
			this->labelStatusActPos0->Size = System::Drawing::Size(15, 15);
			this->labelStatusActPos0->TabIndex = 17;
			this->labelStatusActPos0->Text = L"0";
			// 
			// labelStatusPosCmd0
			// 
			this->labelStatusPosCmd0->AutoSize = true;
			this->labelStatusPosCmd0->ForeColor = System::Drawing::Color::GreenYellow;
			this->labelStatusPosCmd0->Location = System::Drawing::Point(125, 54);
			this->labelStatusPosCmd0->Name = L"labelStatusPosCmd0";
			this->labelStatusPosCmd0->Size = System::Drawing::Size(15, 15);
			this->labelStatusPosCmd0->TabIndex = 13;
			this->labelStatusPosCmd0->Text = L"0";
			// 
			// labelStatusOP0
			// 
			this->labelStatusOP0->AutoSize = true;
			this->labelStatusOP0->BackColor = System::Drawing::Color::Green;
			this->labelStatusOP0->Location = System::Drawing::Point(214, 20);
			this->labelStatusOP0->Name = L"labelStatusOP0";
			this->labelStatusOP0->Size = System::Drawing::Size(64, 15);
			this->labelStatusOP0->TabIndex = 9;
			this->labelStatusOP0->Text = L"OFFLINE";
			// 
			// label8
			// 
			this->label8->AutoSize = true;
			this->label8->Font = (gcnew System::Drawing::Font(L"Gulim", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(129)));
			this->label8->Location = System::Drawing::Point(274, 82);
			this->label8->Name = L"label8";
			this->label8->Size = System::Drawing::Size(53, 15);
			this->label8->TabIndex = 8;
			this->label8->Text = L"ActVel";
			// 
			// label7
			// 
			this->label7->AutoSize = true;
			this->label7->Font = (gcnew System::Drawing::Font(L"Gulim", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(129)));
			this->label7->Location = System::Drawing::Point(266, 54);
			this->label7->Name = L"label7";
			this->label7->Size = System::Drawing::Size(61, 15);
			this->label7->TabIndex = 7;
			this->label7->Text = L"VelCmd";
			// 
			// label6
			// 
			this->label6->AutoSize = true;
			this->label6->Font = (gcnew System::Drawing::Font(L"Gulim", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(129)));
			this->label6->Location = System::Drawing::Point(32, 82);
			this->label6->Name = L"label6";
			this->label6->Size = System::Drawing::Size(61, 15);
			this->label6->TabIndex = 6;
			this->label6->Text = L"ActPos";
			// 
			// label5
			// 
			this->label5->AutoSize = true;
			this->label5->Font = (gcnew System::Drawing::Font(L"Gulim", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(129)));
			this->label5->Location = System::Drawing::Point(24, 54);
			this->label5->Name = L"label5";
			this->label5->Size = System::Drawing::Size(69, 15);
			this->label5->TabIndex = 5;
			this->label5->Text = L"PosCmd";
			// 
			// labelStatusAxis0
			// 
			this->labelStatusAxis0->AutoSize = true;
			this->labelStatusAxis0->Location = System::Drawing::Point(120, 21);
			this->labelStatusAxis0->Name = L"labelStatusAxis0";
			this->labelStatusAxis0->Size = System::Drawing::Size(44, 15);
			this->labelStatusAxis0->TabIndex = 0;
			this->labelStatusAxis0->Text = L"Axis0";
			// 
			// groupBox14
			// 
			this->groupBox14->BackColor = System::Drawing::Color::Black;
			this->groupBox14->Controls->Add(this->label_agv_angleValueDecimal);
			this->groupBox14->Controls->Add(this->label_agv_warningValue);
			this->groupBox14->Controls->Add(this->label_agv_yValue);
			this->groupBox14->Controls->Add(this->label_agv_xValue);
			this->groupBox14->Controls->Add(this->label_agv_angleValue);
			this->groupBox14->Controls->Add(this->label_agv_tagValue);
			this->groupBox14->Controls->Add(this->label53);
			this->groupBox14->Controls->Add(this->label52);
			this->groupBox14->Controls->Add(this->label51);
			this->groupBox14->Controls->Add(this->label50);
			this->groupBox14->Controls->Add(this->label47);
			this->groupBox14->Location = System::Drawing::Point(19, 110);
			this->groupBox14->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->groupBox14->Name = L"groupBox14";
			this->groupBox14->Padding = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->groupBox14->Size = System::Drawing::Size(483, 134);
			this->groupBox14->TabIndex = 179;
			this->groupBox14->TabStop = false;
			// 
			// label_agv_angleValueDecimal
			// 
			this->label_agv_angleValueDecimal->AutoSize = true;
			this->label_agv_angleValueDecimal->ForeColor = System::Drawing::Color::Lime;
			this->label_agv_angleValueDecimal->Location = System::Drawing::Point(144, 74);
			this->label_agv_angleValueDecimal->Name = L"label_agv_angleValueDecimal";
			this->label_agv_angleValueDecimal->Size = System::Drawing::Size(23, 15);
			this->label_agv_angleValueDecimal->TabIndex = 173;
			this->label_agv_angleValueDecimal->Text = L"00";
			// 
			// label_agv_warningValue
			// 
			this->label_agv_warningValue->AutoSize = true;
			this->label_agv_warningValue->Font = (gcnew System::Drawing::Font(L"Gulim", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(129)));
			this->label_agv_warningValue->ForeColor = System::Drawing::Color::Lime;
			this->label_agv_warningValue->Location = System::Drawing::Point(329, 21);
			this->label_agv_warningValue->Name = L"label_agv_warningValue";
			this->label_agv_warningValue->Size = System::Drawing::Size(25, 15);
			this->label_agv_warningValue->TabIndex = 172;
			this->label_agv_warningValue->Text = L"--";
			// 
			// label_agv_yValue
			// 
			this->label_agv_yValue->AutoSize = true;
			this->label_agv_yValue->ForeColor = System::Drawing::Color::Lime;
			this->label_agv_yValue->Location = System::Drawing::Point(225, 99);
			this->label_agv_yValue->Name = L"label_agv_yValue";
			this->label_agv_yValue->Size = System::Drawing::Size(23, 15);
			this->label_agv_yValue->TabIndex = 171;
			this->label_agv_yValue->Text = L"00";
			// 
			// label_agv_xValue
			// 
			this->label_agv_xValue->AutoSize = true;
			this->label_agv_xValue->ForeColor = System::Drawing::Color::Lime;
			this->label_agv_xValue->Location = System::Drawing::Point(69, 99);
			this->label_agv_xValue->Name = L"label_agv_xValue";
			this->label_agv_xValue->Size = System::Drawing::Size(23, 15);
			this->label_agv_xValue->TabIndex = 170;
			this->label_agv_xValue->Text = L"00";
			// 
			// label_agv_angleValue
			// 
			this->label_agv_angleValue->AutoSize = true;
			this->label_agv_angleValue->ForeColor = System::Drawing::Color::Lime;
			this->label_agv_angleValue->Location = System::Drawing::Point(227, 71);
			this->label_agv_angleValue->Name = L"label_agv_angleValue";
			this->label_agv_angleValue->Size = System::Drawing::Size(23, 15);
			this->label_agv_angleValue->TabIndex = 169;
			this->label_agv_angleValue->Text = L"00";
			// 
			// label_agv_tagValue
			// 
			this->label_agv_tagValue->AutoSize = true;
			this->label_agv_tagValue->Font = (gcnew System::Drawing::Font(L"Gulim", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(129)));
			this->label_agv_tagValue->ForeColor = System::Drawing::Color::Lime;
			this->label_agv_tagValue->Location = System::Drawing::Point(142, 42);
			this->label_agv_tagValue->Name = L"label_agv_tagValue";
			this->label_agv_tagValue->Size = System::Drawing::Size(25, 15);
			this->label_agv_tagValue->TabIndex = 168;
			this->label_agv_tagValue->Text = L"00";
			// 
			// label53
			// 
			this->label53->AutoSize = true;
			this->label53->ForeColor = System::Drawing::Color::White;
			this->label53->Location = System::Drawing::Point(83, 42);
			this->label53->Name = L"label53";
			this->label53->Size = System::Drawing::Size(31, 15);
			this->label53->TabIndex = 167;
			this->label53->Text = L"Tag";
			// 
			// label52
			// 
			this->label52->AutoSize = true;
			this->label52->ForeColor = System::Drawing::Color::White;
			this->label52->Location = System::Drawing::Point(173, 99);
			this->label52->Name = L"label52";
			this->label52->Size = System::Drawing::Size(15, 15);
			this->label52->TabIndex = 166;
			this->label52->Text = L"Y";
			// 
			// label51
			// 
			this->label51->AutoSize = true;
			this->label51->ForeColor = System::Drawing::Color::White;
			this->label51->Location = System::Drawing::Point(21, 99);
			this->label51->Name = L"label51";
			this->label51->Size = System::Drawing::Size(16, 15);
			this->label51->TabIndex = 165;
			this->label51->Text = L"X";
			// 
			// label50
			// 
			this->label50->AutoSize = true;
			this->label50->ForeColor = System::Drawing::Color::White;
			this->label50->Location = System::Drawing::Point(73, 71);
			this->label50->Name = L"label50";
			this->label50->Size = System::Drawing::Size(43, 15);
			this->label50->TabIndex = 164;
			this->label50->Text = L"Angle";
			// 
			// label47
			// 
			this->label47->AutoSize = true;
			this->label47->Font = (gcnew System::Drawing::Font(L"Gadugi", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
			this->label47->ForeColor = System::Drawing::Color::Yellow;
			this->label47->Location = System::Drawing::Point(6, 0);
			this->label47->Name = L"label47";
			this->label47->Size = System::Drawing::Size(103, 19);
			this->label47->TabIndex = 164;
			this->label47->Text = L"Position Data";
			// 
			// button_start_motion
			// 
			this->button_start_motion->Font = (gcnew System::Drawing::Font(L"Gulim", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(129)));
			this->button_start_motion->Location = System::Drawing::Point(805, 288);
			this->button_start_motion->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->button_start_motion->Name = L"button_start_motion";
			this->button_start_motion->Size = System::Drawing::Size(136, 56);
			this->button_start_motion->TabIndex = 180;
			this->button_start_motion->Text = L"Start Motion";
			this->button_start_motion->UseVisualStyleBackColor = true;
			this->button_start_motion->Click += gcnew System::EventHandler(this, &MainForm::button_start_motion_Click);
			// 
			// textBoxDeviceName
			// 
			this->textBoxDeviceName->Font = (gcnew System::Drawing::Font(L"Gulim", 10.2F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(129)));
			this->textBoxDeviceName->Location = System::Drawing::Point(26, 14);
			this->textBoxDeviceName->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->textBoxDeviceName->Name = L"textBoxDeviceName";
			this->textBoxDeviceName->Size = System::Drawing::Size(140, 27);
			this->textBoxDeviceName->TabIndex = 181;
			this->textBoxDeviceName->Text = L"Lab2mDevice";
			// 
			// button_stop_motion
			// 
			this->button_stop_motion->Font = (gcnew System::Drawing::Font(L"Gulim", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(129)));
			this->button_stop_motion->Location = System::Drawing::Point(739, 288);
			this->button_stop_motion->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->button_stop_motion->Name = L"button_stop_motion";
			this->button_stop_motion->Size = System::Drawing::Size(60, 56);
			this->button_stop_motion->TabIndex = 182;
			this->button_stop_motion->Text = L"STOP";
			this->button_stop_motion->UseVisualStyleBackColor = true;
			this->button_stop_motion->Click += gcnew System::EventHandler(this, &MainForm::button_stop_motion_Click);
			// 
			// groupBox1
			// 
			this->groupBox1->Controls->Add(this->groupBox14);
			this->groupBox1->Controls->Add(this->button_connect_PGV);
			this->groupBox1->Controls->Add(this->button_open_port);
			this->groupBox1->Location = System::Drawing::Point(552, 12);
			this->groupBox1->Name = L"groupBox1";
			this->groupBox1->Size = System::Drawing::Size(530, 255);
			this->groupBox1->TabIndex = 183;
			this->groupBox1->TabStop = false;
			// 
			// groupBox2
			// 
			this->groupBox2->Controls->Add(this->groupBox4);
			this->groupBox2->Controls->Add(this->button_servo);
			this->groupBox2->Controls->Add(this->button_communication);
			this->groupBox2->Controls->Add(this->textBoxDeviceName);
			this->groupBox2->Controls->Add(this->button_create_device);
			this->groupBox2->Location = System::Drawing::Point(16, 12);
			this->groupBox2->Name = L"groupBox2";
			this->groupBox2->Size = System::Drawing::Size(530, 255);
			this->groupBox2->TabIndex = 184;
			this->groupBox2->TabStop = false;
			// 
			// MainForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(8, 15);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1094, 393);
			this->Controls->Add(this->groupBox2);
			this->Controls->Add(this->groupBox1);
			this->Controls->Add(this->button_stop_motion);
			this->Controls->Add(this->richTextBoxMessage);
			this->Controls->Add(this->button_start_motion);
			this->Controls->Add(this->button_clear_message);
			this->Controls->Add(this->button6);
			this->Controls->Add(this->button1);
			this->Name = L"MainForm";
			this->Text = L"PGV sensor controlled motion";
			this->Load += gcnew System::EventHandler(this, &MainForm::MainForm_Load);
			this->groupBox4->ResumeLayout(false);
			this->groupBox4->PerformLayout();
			this->groupBox14->ResumeLayout(false);
			this->groupBox14->PerformLayout();
			this->groupBox1->ResumeLayout(false);
			this->groupBox2->ResumeLayout(false);
			this->groupBox2->PerformLayout();
			this->ResumeLayout(false);

		}
#pragma endregion
	//----------------------------------------------------------------------------
	// Custom Functions.
	//----------------------------------------------------------------------------
	//Control servo On/Off
		Void servoOnOff(int axis, bool controlFlag, System::Windows::Forms::Button^ btn) {
			int onOffStatus = 0;

			if (controlFlag) {
				btn->BackColor = System::Drawing::Color::GreenYellow;
				onOffStatus = 1;
			}
			else {
				btn->BackColor = System::Drawing::Color::Ivory;
				onOffStatus = 0;
			}

			err = wmxlib_cm.axisControl->SetServoOn(axis, onOffStatus);


			err = wmxlib_cm.axisControl->SetServoOn(axis, onOffStatus); //Set axis 0 servo off
			if (err != ErrorCode::None) {
				wmxlib_cm.ErrorToString(err, errString, sizeof(errString));
				richTextBoxMessage->Text = "Failed to set servo off. Error=%d (%s)\n", err, errString;
			}
		}

	private: System::Void timer_Axis(System::Object^ sender, System::EventArgs^ e) {
		if (servoOnOff0) {
			cmAxis = &CmStatus.axesStatus[0];
			wmxlib_cm.GetStatus(&CmStatus);
			labelStatusOP0->Text = OP_STATE(cmAxis->opState);
			labelStatusPosCmd0->Text = cmAxis->posCmd.ToString();
			labelStatusActPos0->Text = cmAxis->actualPos.ToString();
			labelStatusVelCmd0->Text = cmAxis->velocityCmd.ToString();
			labelStatusActVel0->Text = cmAxis->actualVelocity.ToString();
		}
	}

	private: System::Void timer_PGV(System::Object^ sender, System::EventArgs^ e) {
		if (PGVStart) {
			//////Warning value
			std::cout << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(receivedData[0]) << std::endl;
			int warningInt = static_cast<int>(receivedData[0]);
			if (warningInt == 2) {

				isAGVTagPosition = false;

				label_agv_tagValue->Text = "No Position";
				label_agv_tagValue->ForeColor = System::Drawing::Color::Red;
			}
			//////Tag Value
			else {
				isAGVTagPosition = true;

				std::cout << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(receivedData[14]) << std::endl;
				std::cout << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(receivedData[15]) << std::endl;
				std::cout << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(receivedData[16]) << std::endl;
				std::cout << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(receivedData[17]) << std::endl;

				int hexTag1 = static_cast<int>(receivedData[14]);
				int hexTag2 = static_cast<int>(receivedData[15]);
				int hexTag3 = static_cast<int>(receivedData[16]);
				int hexTag4 = static_cast<int>(receivedData[17]);

				// Convert integers to hexadecimal strings
				std::stringstream ss;
				ss << std::hex << std::setw(2) << std::setfill('0') << hexTag1;
				std::string strTag1 = ss.str();

				ss.str("");
				ss << std::hex << std::setw(2) << std::setfill('0') << hexTag2;
				std::string strTag2 = ss.str();

				ss.str("");
				ss << std::hex << std::setw(2) << std::setfill('0') << hexTag3;
				std::string strTag3 = ss.str();

				ss.str("");
				ss << std::hex << std::setw(2) << std::setfill('0') << hexTag4;
				std::string strTag4 = ss.str();

				System::String^ fullTagValue = std::stoi(strTag1, nullptr, 16).ToString() + std::stoi(strTag2, nullptr, 16).ToString() + std::stoi(strTag3, nullptr, 16).ToString() + std::stoi(strTag4, nullptr, 16).ToString();


				label_agv_tagValue->Text = fullTagValue;
				label_agv_tagValue->ForeColor = System::Drawing::Color::Lime;

			}
		}
	
	}

	private: System::Void threadReadQR() {
		while (true) {
			if (isAGVTagPosition) {
				
				Motion::JogCommand jogCommand = Motion::JogCommand();
				jogCommand.profile.type = ProfileType::SCurve;
				jogCommand.axis = 0;
				jogCommand.profile.velocity = slowJogSpeed;
				jogCommand.profile.acc = slowAccelSpeed;
				jogCommand.profile.dec = slowDecelSpeed;
				wmxlib_cm.motion->StartJog(&jogCommand);

				this->thrAwaitSlow->Abort();
			}
			else {
				//richTextBoxMessage->Text = " ";
				//Sleep(100);
				//richTextBoxMessage->Text = "Reading";
				// nothing TODO
			}
		}
	}

	private: System::Void threadReadIO() {
		while (true) {


		}
	}

	private: System::Boolean portConnection() {
		char createdPort[50];

		// Open the serial port
		//hSerial = CreateFile(_T("COM1~9"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
		hSerial = CreateFile(_T("\\\\.\\COM12"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);

		if (hSerial == INVALID_HANDLE_VALUE) {
			richTextBoxMessage->Text = "Error: Unable to open serial port." + GetLastError().ToString();
			button_open_port->Text = "Open Port";
			return false;
		}

		// Set serial port parameters
		dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
		if (!GetCommState(hSerial, &dcbSerialParams)) {
			richTextBoxMessage->Text = "Error: Error: Unable to get serial port state." + GetLastError().ToString();
			CloseHandle(hSerial);
			button_open_port->Text = "Open Port";
			return false;
		}

		dcbSerialParams.BaudRate = CBR_115200; // Set your baudrate here
		dcbSerialParams.ByteSize = 8;
		dcbSerialParams.StopBits = ONESTOPBIT;
		dcbSerialParams.Parity = EVENPARITY;

		if (!SetCommState(hSerial, &dcbSerialParams)) {
			richTextBoxMessage->Text = "Error: Unable to set serial port state. Error code: " + GetLastError().ToString();
			CloseHandle(hSerial);
			button_open_port->Text = "Open Port";
			return false;
		}

		// Set timeouts
		timeouts.ReadIntervalTimeout = 1000;
		timeouts.ReadTotalTimeoutConstant = 1000;
		timeouts.ReadTotalTimeoutMultiplier = 1;
		timeouts.WriteTotalTimeoutConstant = 1000;
		timeouts.WriteTotalTimeoutMultiplier = 1;

		if (!SetCommTimeouts(hSerial, &timeouts)) {
			richTextBoxMessage->Text = "Error: Unable to set timeouts. Error code: " + GetLastError().ToString();
			CloseHandle(hSerial);
			button_open_port->Text = "Open Port";
			return false;
		}

		//After port is successfully opened and request Telegram
		// (Write/Send) Request telegram ( Choose Lane LEFT)
		if (!WriteFile(hSerial, dataToRequestLeftLane, sizeof(dataToRequestLeftLane), &bytesWritten, NULL)) {
			richTextBoxMessage->Text = "Error: Unable to write to serial port. Error code: " + GetLastError();
			CloseHandle(hSerial);
			button_open_port->Text = "Open Port";
			return false;
		}

		button_open_port->Text = "Close Port";
		return true;
	}

	private: System::Void PGVDataReading() {
		while (true) {

			if (!portState) {
				return;
			}
			if (!PGVStart) {
				return;
			}

			// (Write/Send) Request telegram
			if (!WriteFile(hSerial, dataToRequest, sizeof(dataToRequest), &bytesWritten, NULL)) {
				richTextBoxMessage->Text = "Error: Unable to write to serial port. Error code: " + GetLastError();
				CloseHandle(hSerial);
			}

			// Read response from the serial port
			if (!ReadFile(hSerial, receivedData, sizeof(receivedData) - 1, &bytesRead, NULL)) {
				richTextBoxMessage->Text = "Error: Unable to read from serial port. Error code: " + GetLastError();
				CloseHandle(hSerial);
			}

			receivedData[bytesRead] = '\0'; // Null-terminate the received data Hex 21bytes from PGV is receivingin here

			// Receive the response in hexadecimal format
			//std::cout << "Response in Hex: ";
			//for (DWORD i = 0; i < bytesRead; ++i) {
			//	std::cout << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(receivedData[i]) << " ";
			//}
			////////////////////////////////////////

			// std::cout << std::endl;
			Sleep(sensorDataInterval);
		}
	}



	//----------------------------------------------------------------------------
	// Event Functions.
	//----------------------------------------------------------------------------
	private: System::Void MainForm_Load(System::Object^ sender, System::EventArgs^ e) {

		// Set Timer to show status of the axis
		Timer^ timerAxis0 = gcnew Timer();
		timerAxis0->Interval = sensorDataInterval;
		timerAxis0->Tick += gcnew System::EventHandler(this, &MainForm::timer_Axis);
		timerAxis0->Start();

		// Set Timer to show status of PGV
		Timer^ timerPGV0 = gcnew Timer();
		timerPGV0->Interval = sensorDataInterval;
		timerPGV0->Tick += gcnew System::EventHandler(this, &MainForm::timer_PGV);
		timerPGV0->Start();

	}

	private: System::Void button_create_device_Click(System::Object^ sender, System::EventArgs^ e) {

		char createdName[50];
		wmx3Api::DevicesInfoA devInfo;

		Device.CreateDevice(pathDir, DeviceType::DeviceTypeNormal, INFINITE);

		std::string stdDeviceName = msclr::interop::marshal_as<std::string>(this->textBoxDeviceName->Text);
		strcpy(createdName, stdDeviceName.c_str());

		Device.SetDeviceName(createdName);

		// Get created device state.
		Device.GetAllDevices(&devInfo);

		// Enable Communication button.
		this->button_communication->Enabled = true;
	}

	private: System::Void button_communication_Click(System::Object^ sender, System::EventArgs^ e) {
		communicationOnOff = !communicationOnOff;
		if (communicationOnOff) {
			Device.StartCommunication(INFINITE);
			this->button_communication->BackColor = System::Drawing::Color::GreenYellow;
			this->button_communication->Text = "Communication";
		}
		else {
			Device.StopCommunication(INFINITE);
			this->button_communication->BackColor = System::Drawing::Color::Ivory;
			this->button_communication->Text = "Stopped";
		}
	}
	private: System::Void button_servo_Click(System::Object^ sender, System::EventArgs^ e) {
		if (communicationOnOff) {
			servoOnOff0 = !servoOnOff0;
			servoOnOff(0, servoOnOff0, this->button_servo);
		}
		else {
			richTextBoxMessage->Text = "Please Communication first!";
		}
	}
	private: System::Void button_clear_message_Click(System::Object^ sender, System::EventArgs^ e) {
			  richTextBoxMessage->Text = "";
	}


	private: System::Void button_stop_motion_Click(System::Object^ sender, System::EventArgs^ e) {
		wmxlib_cm.motion->ExecTimedStop(0, 1); // stop at 1ms

		// Test
		// isAGVTagPosition = true;
	}
	
	private: System::Void button_start_motion_Click(System::Object^ sender, System::EventArgs^ e) {
		Motion::JogCommand jogCommand = Motion::JogCommand();
		jogCommand.profile.type = ProfileType::SCurve;
		jogCommand.axis = 0;
		jogCommand.profile.velocity = normalJogSpeed;
		jogCommand.profile.acc = normalAccelSpeed;
		jogCommand.profile.dec = normalDecelSpeed;
		wmxlib_cm.motion->StartJog(&jogCommand);

		// A thread is called to change the SLOW speed when QR code is read
		this->thrAwaitSlow = gcnew System::Threading::Thread(gcnew System::Threading::ThreadStart(this, &MainForm::threadReadQR));
		this->thrAwaitSlow->Start();
	}
	private: System::Void button_open_port_Click(System::Object^ sender, System::EventArgs^ e) {
		if (portState) {
			CloseHandle(hSerial);
			portState = false;
			button_open_port->Text = "Open Port";

			PGVStart = false;
			button_connect_PGV->BackColor = System::Drawing::Color::White;
		}
		else {
			portState = portConnection();
		}
	}

	private: System::Void button_connect_PGV_Click(System::Object^ sender, System::EventArgs^ e) {
		if (!portState) {
			richTextBoxMessage->Text = "PGV port is not connected yet!";
			return;
		}

		PGVStart = !PGVStart;
		if (PGVStart) {
			button_connect_PGV->BackColor = System::Drawing::Color::GreenYellow;
			button_open_port->Enabled = false;
		}
		else {
			button_connect_PGV->BackColor = System::Drawing::Color::White;
			button_open_port->Enabled = true;
		}


		// Continuously Receiving by thread
		if (portState) {
			this->thrPGVDataReceiving = gcnew System::Threading::Thread(gcnew System::Threading::ThreadStart(this, &MainForm::PGVDataReading));
			this->thrPGVDataReceiving->Start();
		}
	}
};
}
