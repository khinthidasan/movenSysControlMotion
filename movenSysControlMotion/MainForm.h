#pragma once

//Library from MovenSys WOS
#include <WMX3Api.h>
#include <WMX3Funcs.h>
#include <CoreMotionApi.h>
#include <EcApi.h>
#include <IOApi.h>

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
	private: System::Windows::Forms::PictureBox^ pictureBox1;
	protected:
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
			System::ComponentModel::ComponentResourceManager^ resources = (gcnew System::ComponentModel::ComponentResourceManager(MainForm::typeid));
			this->pictureBox1 = (gcnew System::Windows::Forms::PictureBox());
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
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->BeginInit();
			this->groupBox4->SuspendLayout();
			this->groupBox14->SuspendLayout();
			this->SuspendLayout();
			// 
			// pictureBox1
			// 
			this->pictureBox1->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"pictureBox1.Image")));
			this->pictureBox1->Location = System::Drawing::Point(12, 11);
			this->pictureBox1->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->pictureBox1->Name = L"pictureBox1";
			this->pictureBox1->Size = System::Drawing::Size(192, 89);
			this->pictureBox1->SizeMode = System::Windows::Forms::PictureBoxSizeMode::StretchImage;
			this->pictureBox1->TabIndex = 1;
			this->pictureBox1->TabStop = false;
			// 
			// richTextBoxMessage
			// 
			this->richTextBoxMessage->Location = System::Drawing::Point(210, 11);
			this->richTextBoxMessage->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->richTextBoxMessage->Name = L"richTextBoxMessage";
			this->richTextBoxMessage->Size = System::Drawing::Size(612, 89);
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
			this->button_create_device->Location = System::Drawing::Point(12, 125);
			this->button_create_device->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->button_create_device->Name = L"button_create_device";
			this->button_create_device->Size = System::Drawing::Size(136, 59);
			this->button_create_device->TabIndex = 12;
			this->button_create_device->Text = L"Create Device";
			this->button_create_device->UseVisualStyleBackColor = true;
			// 
			// button_communication
			// 
			this->button_communication->Enabled = false;
			this->button_communication->Font = (gcnew System::Drawing::Font(L"Gulim", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(129)));
			this->button_communication->Location = System::Drawing::Point(191, 125);
			this->button_communication->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->button_communication->Name = L"button_communication";
			this->button_communication->Size = System::Drawing::Size(136, 59);
			this->button_communication->TabIndex = 11;
			this->button_communication->Text = L"Stopped";
			this->button_communication->UseVisualStyleBackColor = true;
			// 
			// button_clear_message
			// 
			this->button_clear_message->Font = (gcnew System::Drawing::Font(L"Gulim", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(129)));
			this->button_clear_message->Location = System::Drawing::Point(839, 8);
			this->button_clear_message->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->button_clear_message->Name = L"button_clear_message";
			this->button_clear_message->Size = System::Drawing::Size(62, 26);
			this->button_clear_message->TabIndex = 174;
			this->button_clear_message->Text = L"Clear Message";
			this->button_clear_message->UseVisualStyleBackColor = true;
			// 
			// button_open_port
			// 
			this->button_open_port->Font = (gcnew System::Drawing::Font(L"Gulim", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(129)));
			this->button_open_port->Location = System::Drawing::Point(640, 125);
			this->button_open_port->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->button_open_port->Name = L"button_open_port";
			this->button_open_port->Size = System::Drawing::Size(136, 59);
			this->button_open_port->TabIndex = 176;
			this->button_open_port->Text = L"Open Port";
			this->button_open_port->UseVisualStyleBackColor = true;
			// 
			// button_connect_PGV
			// 
			this->button_connect_PGV->Font = (gcnew System::Drawing::Font(L"Gulim", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(129)));
			this->button_connect_PGV->Location = System::Drawing::Point(803, 125);
			this->button_connect_PGV->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->button_connect_PGV->Name = L"button_connect_PGV";
			this->button_connect_PGV->Size = System::Drawing::Size(136, 59);
			this->button_connect_PGV->TabIndex = 175;
			this->button_connect_PGV->Text = L"PGV";
			this->button_connect_PGV->UseVisualStyleBackColor = true;
			// 
			// button_servo
			// 
			this->button_servo->Font = (gcnew System::Drawing::Font(L"Gulim", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(129)));
			this->button_servo->Location = System::Drawing::Point(359, 125);
			this->button_servo->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->button_servo->Name = L"button_servo";
			this->button_servo->Size = System::Drawing::Size(136, 59);
			this->button_servo->TabIndex = 177;
			this->button_servo->Text = L"Servo On";
			this->button_servo->UseVisualStyleBackColor = true;
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
			this->groupBox4->Location = System::Drawing::Point(12, 197);
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
			this->labelStatusActVel0->Location = System::Drawing::Point(420, 82);
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
			this->labelStatusVelCmd0->Location = System::Drawing::Point(420, 54);
			this->labelStatusVelCmd0->Name = L"labelStatusVelCmd0";
			this->labelStatusVelCmd0->Size = System::Drawing::Size(15, 15);
			this->labelStatusVelCmd0->TabIndex = 21;
			this->labelStatusVelCmd0->Text = L"0";
			// 
			// labelStatusActPos0
			// 
			this->labelStatusActPos0->AutoSize = true;
			this->labelStatusActPos0->ForeColor = System::Drawing::Color::GreenYellow;
			this->labelStatusActPos0->Location = System::Drawing::Point(189, 82);
			this->labelStatusActPos0->Name = L"labelStatusActPos0";
			this->labelStatusActPos0->Size = System::Drawing::Size(15, 15);
			this->labelStatusActPos0->TabIndex = 17;
			this->labelStatusActPos0->Text = L"0";
			// 
			// labelStatusPosCmd0
			// 
			this->labelStatusPosCmd0->AutoSize = true;
			this->labelStatusPosCmd0->ForeColor = System::Drawing::Color::GreenYellow;
			this->labelStatusPosCmd0->Location = System::Drawing::Point(189, 54);
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
			this->groupBox14->Location = System::Drawing::Point(525, 197);
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
			this->button_start_motion->Location = System::Drawing::Point(435, 363);
			this->button_start_motion->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->button_start_motion->Name = L"button_start_motion";
			this->button_start_motion->Size = System::Drawing::Size(136, 46);
			this->button_start_motion->TabIndex = 180;
			this->button_start_motion->Text = L"Start Motion";
			this->button_start_motion->UseVisualStyleBackColor = true;
			// 
			// MainForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(8, 15);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1020, 418);
			this->Controls->Add(this->button_start_motion);
			this->Controls->Add(this->groupBox14);
			this->Controls->Add(this->groupBox4);
			this->Controls->Add(this->button_servo);
			this->Controls->Add(this->button_open_port);
			this->Controls->Add(this->button_connect_PGV);
			this->Controls->Add(this->button_clear_message);
			this->Controls->Add(this->button_create_device);
			this->Controls->Add(this->button_communication);
			this->Controls->Add(this->richTextBoxMessage);
			this->Controls->Add(this->button6);
			this->Controls->Add(this->button1);
			this->Controls->Add(this->pictureBox1);
			this->Name = L"MainForm";
			this->Text = L"PGV sensor controlled motion";
			this->Load += gcnew System::EventHandler(this, &MainForm::MainForm_Load);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->EndInit();
			this->groupBox4->ResumeLayout(false);
			this->groupBox4->PerformLayout();
			this->groupBox14->ResumeLayout(false);
			this->groupBox14->PerformLayout();
			this->ResumeLayout(false);

		}
#pragma endregion
	private: System::Void MainForm_Load(System::Object^ sender, System::EventArgs^ e) {
	}
	};
}
