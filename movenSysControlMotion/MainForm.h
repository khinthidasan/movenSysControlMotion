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
			this->SuspendLayout();
			// 
			// MainForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(8, 15);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1251, 706);
			this->Name = L"MainForm";
			this->Text = L"MovenSys Motion Control";
			this->Load += gcnew System::EventHandler(this, &MainForm::MainForm_Load);
			this->ResumeLayout(false);

		}
#pragma endregion
	private: System::Void MainForm_Load(System::Object^ sender, System::EventArgs^ e) {
	}
	};
}
