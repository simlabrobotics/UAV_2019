#ifndef __UAV_CONF_H__
#define __UAV_CONF_H__

#include <rlab/type.h>
#include <rxSDK\/rxUserProperty.h>
#include <rlab/utils/rParseUtil.h>

#define UAVCONF_READ_PROP_BOOLEAN(prop_name, var_name)  \
	prop = getProperty(prop_name);						\
		if (prop) {										\
			if (_tcsicmp(prop, _T("true")) == 0 ||		\
				_tcsicmp(prop, _T("yes")) == 0)			\
				(##var_name) = true;					\
			else										\
				(##var_name) = false;					\
		}

#define UAVCONF_READ_PROP_INT(prop_name, var_name)		\
	prop = getProperty(prop_name);						\
	if (prop) (##var_name) = _tstoi(prop)

#define UAVCONF_READ_PROP_REAL(prop_name, var_name)		\
	prop = getProperty(prop_name);						\
	if (prop) (##var_name) = _tstof(prop)

#define MAX_UAV_NUM (2)

class UAVConf : public rxUserProperty
{
public:
	UAVConf() 
		: _bLoaded(false)
		, _delT(0.005)
		, _use_AMBSDynamics(true)
		, _net_port_simulation(5150)
		, _debug_control_local_enable(false)
	{
		_enable_eml = false;
		_eml_filepath = _T("");
		_eml_name = _T("Environment");
		_eml_r0.setZero();
		_eml_R0.setIdentity();

		for (int uav_no = 0; uav_no < MAX_UAV_NUM; uav_no++)
		{
			_enable_aml[uav_no] = false;
			//_aml_filepath[uav_no] = _T("");
			//_aml_name[uav_no] = _T("MyRobot");
			_aml_isStatic[uav_no] = false;
			_aml_R0[uav_no].setIdentity();
			_aml_r0[uav_no].setZero();

			_net_port_control[uav_no] = 5250 + i + 1;

			_plot_position_enabled[uav_no] = false;
			_plot_velocity_enabled[uav_no] = false;
			_plot_acc_sensor_enabled[uav_no] = false;
			_plot_gyro_sensor_enabled[uav_no] = false;
			_plot_gps_sensor_enabled[uav_no] = false;
			_plot_slippage_enabled[uav_no] = false;
			_plot_slippage_tracked_enabled[uav_no] = false;
			_plot_lateral_force_enabled[uav_no] = false;
		}
	}
		
	~UAVConf() {}

	void parseConfig(const string_type& filepath)
	{
		loadProperty(filepath);

		const TCHAR* prop;
		TCHAR prop_name[256];

		prop = getProperty(_T("time_step"));
		if (prop)
			_delT = _tstof(prop);

		UAVCONF_READ_PROP_BOOLEAN(_T("use_AMBSDynamics"), _use_AMBSDynamics);

		for (int uav_no = 0; uav_no < MAX_UAV_NUM; uav_no++)
		{
			_stprintf_s(prop_name, 256, _T("aml%d_enable"), uav_no + 1);
			UAVCONF_READ_PROP_BOOLEAN(prop_name, _enable_aml[uav_no]);

			_stprintf_s(prop_name, 256, _T("aml%d_isStatic"), uav_no + 1);
			UAVCONF_READ_PROP_BOOLEAN(prop_name, _aml_isStatic[uav_no]);

			_stprintf_s(prop_name, 256, _T("aml%d_name"), uav_no + 1);
			prop = getProperty(prop_name);
			if (prop)
				_aml_name[uav_no] = prop;

			_stprintf_s(prop_name, 256, _T("aml%d_path"), uav_no + 1);
			prop = getProperty(prop_name);
			if (prop)
				_aml_filepath[uav_no] = prop;

			_stprintf_s(prop_name, 256, _T("aml%d_r0"), uav_no + 1);
			prop = getProperty(prop_name);
			if (prop) {
				float r[3];
				parse_vector(r, 3, prop);
				for (int i = 0; i<3; i++)
					_aml_r0[uav_no][i] = r[i];
			}

			_stprintf_s(prop_name, 256, _T("aml%d_R0"), uav_no + 1);
			prop = getProperty(prop_name);
			if (prop) {
				float R[9];
				parse_vector(R, 9, prop);
				_aml_R0[uav_no] << R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8];
			}

			_stprintf_s(prop_name, 256, _T("net_port_control%d"), uav_no + 1);
			UAVCONF_READ_PROP_INT(prop_name, _net_port_control[uav_no]);

			_stprintf_s(prop_name, 256, _T("plot%d_position"), uav_no + 1);
			UAVCONF_READ_PROP_BOOLEAN(prop_name, _plot_position_enabled[uav_no]);

			_stprintf_s(prop_name, 256, _T("plot%d_velocity"), uav_no + 1);
			UAVCONF_READ_PROP_BOOLEAN(prop_name, _plot_velocity_enabled[uav_no]);

			_stprintf_s(prop_name, 256, _T("plot%d_gyro_sensor"), uav_no + 1);
			UAVCONF_READ_PROP_BOOLEAN(prop_name, _plot_gyro_sensor_enabled[uav_no]);

			_stprintf_s(prop_name, 256, _T("plot%d_acc_sensor"), uav_no + 1);
			UAVCONF_READ_PROP_BOOLEAN(prop_name, _plot_acc_sensor_enabled[uav_no]);

			_stprintf_s(prop_name, 256, _T("plot%d_gps_sensor"), uav_no + 1);
			UAVCONF_READ_PROP_BOOLEAN(prop_name, _plot_gps_sensor_enabled[uav_no]);

			_stprintf_s(prop_name, 256, _T("plot%d_slippage"), uav_no + 1);
			UAVCONF_READ_PROP_BOOLEAN(prop_name, _plot_slippage_enabled[uav_no]);

			_stprintf_s(prop_name, 256, _T("plot%d_slippage_tracked"), uav_no + 1);
			UAVCONF_READ_PROP_BOOLEAN(prop_name, _plot_slippage_tracked_enabled[uav_no]);

			_stprintf_s(prop_name, 256, _T("plot%d_lateral_force"), uav_no + 1);
			UAVCONF_READ_PROP_BOOLEAN(prop_name, _plot_lateral_force_enabled[uav_no]);
		}

		UAVCONF_READ_PROP_BOOLEAN(_T("eml_enable"), _enable_eml);

		prop = getProperty(_T("eml_name"));
		if (prop)
			_eml_name = prop;

		prop = getProperty(_T("eml_path"));
		if (prop)
			_eml_filepath = prop;

		prop = getProperty(_T("eml_r0"));
		if (prop) {
			float r[3];
			parse_vector(r, 3, prop);
			for (int i=0; i<3; i++)
				_eml_r0[i] = r[i];
		}

		prop = getProperty(_T("eml_R0"));
		if (prop) {
			float R[9];
			parse_vector(R, 9, prop);
			_eml_R0(0, 0) = R[0]; _eml_R0(0, 1) = R[1]; _eml_R0(0, 2) = R[2];
			_eml_R0(1, 0) = R[3]; _eml_R0(1, 1) = R[4]; _eml_R0(1, 2) = R[5];
			_eml_R0(2, 0) = R[6]; _eml_R0(2, 1) = R[7]; _eml_R0(2, 2) = R[8];
		}

		UAVCONF_READ_PROP_INT(_T("net_port_simulation"), _net_port_simulation);

		UAVCONF_READ_PROP_BOOLEAN(_T("debug_control_local_enable"), _debug_control_local_enable);

		_bLoaded = true;
	}
	
	bool isLoaded() { return _bLoaded; }

public:
	rTime delT() const { return _delT; }
	bool useAMBSDynamics() const { return _use_AMBSDynamics; }
	
	bool enableAml(unsigned int uav_no) const { return _enable_aml[uav_no]; }
	bool amlIsStatic(unsigned int uav_no) const { return _aml_isStatic[uav_no]; }
	const string_type& amlPath(unsigned int uav_no) const { return _aml_filepath[uav_no]; }
	const string_type& amlName(unsigned int uav_no) const { return _aml_name[uav_no]; }
	const Displacement& aml_r0(unsigned int uav_no) const { return _aml_r0[uav_no]; }
	const Rotation& aml_R0(unsigned int uav_no) const { return _aml_R0[uav_no]; }
	const dVector& aml_q0(unsigned int uav_no) const { return _aml_q0[uav_no]; }
	
	bool enableEml() const { return _enable_eml; }
	const string_type& emlPath() const { return _eml_filepath; }
	const string_type& emlName() const { return _eml_name; }
	const Displacement& eml_r0() const { return _eml_r0; }
	const Rotation& eml_R0() const { return _eml_R0; }

	bool enablePlotPosition(unsigned int uav_no) const { return _plot_position_enabled[uav_no]; }
	bool enablePlotVelocity(unsigned int uav_no) const { return _plot_velocity_enabled[uav_no]; }
	bool enablePlotAccSensor(unsigned int uav_no) const { return _plot_acc_sensor_enabled[uav_no]; }
	bool enablePlotGyroSensor(unsigned int uav_no) const { return _plot_gyro_sensor_enabled[uav_no]; }
	bool enablePlotGPSSensor(unsigned int uav_no) const { return _plot_gps_sensor_enabled[uav_no]; }
	bool enablePlotSlippage(unsigned int uav_no) const { return _plot_slippage_enabled[uav_no]; }
	bool enablePlotSlippageTracked(unsigned int uav_no) const { return _plot_slippage_tracked_enabled[uav_no]; }
	bool enablePlotLateralForce(unsigned int uav_no) const { return _plot_lateral_force_enabled[uav_no]; }
	
	unsigned short netSimulationPort() const { return _net_port_simulation; }
	unsigned short netControlPort(unsigned int uav_no) const { return _net_port_control[uav_no]; }

	bool enableDebugControlLocal() const { return _debug_control_local_enable; }

private:
	bool _bLoaded;

	// application
	rTime _delT;
	bool _use_AMBSDynamics;
	// robot
	bool _enable_aml[MAX_UAV_NUM];
	bool _aml_isStatic[MAX_UAV_NUM];
	string_type _aml_filepath[MAX_UAV_NUM];
	string_type _aml_name[MAX_UAV_NUM];
	Displacement _aml_r0[MAX_UAV_NUM];
	Rotation _aml_R0[MAX_UAV_NUM];
	dVector _aml_q0[MAX_UAV_NUM];
	// environment
	bool _enable_eml;
	string_type _eml_filepath;
	string_type _eml_name;
	Displacement _eml_r0;
	Rotation _eml_R0;
	// plot
	bool _plot_position_enabled[MAX_UAV_NUM];
	bool _plot_velocity_enabled[MAX_UAV_NUM];
	bool _plot_acc_sensor_enabled[MAX_UAV_NUM];
	bool _plot_gyro_sensor_enabled[MAX_UAV_NUM];
	bool _plot_gps_sensor_enabled[MAX_UAV_NUM];
	bool _plot_slippage_enabled[MAX_UAV_NUM];
	bool _plot_slippage_tracked_enabled[MAX_UAV_NUM];
	bool _plot_lateral_force_enabled[MAX_UAV_NUM];
	// network
	unsigned short _net_port_simulation;
	unsigned short _net_port_control[MAX_UAV_NUM];
	// debug
	bool _debug_control_local_enable;
};

#endif // __UAV_CONF_H__
