
#pragma once

#include <Eigen/Core>
#include <cstdlib> 
#include <ctime>   
#include <limits>
#include <cmath>
#include <chrono>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include "types.h"

using namespace std;
namespace chr = std::chrono;


class Locator
{
public:
	
	double convergeTolerance = 0.2;	 
	double residualTolerance = 0.4;	 
	double maxIteration = 20;		 
	double muOffset = 2.0;			 
	double numShrinkRatio = 0.85;	 
	double offsetShrinkRatio = 0.8;	 
	int minMarkerCnt = 3;		 

	
	vector<FieldMarker> fieldMarkers;
	FieldDimensions fieldDimensions;
	Eigen::ArrayXXd hypos;				  
	PoseBox2D constraints;				  
	double offsetX, offsetY, offsetTheta; 
	Pose2D bestPose;					  
	double bestResidual;				  

	void init(FieldDimensions fd, int minMarkerCnt = 4, double residualTolerance = 0.4, double muOffsetParam = 2.0);

	
	void calcFieldMarkers(FieldDimensions fd);


	LocateResult locateRobot(vector<FieldMarker> markers_r, PoseBox2D constraints, int numParticles = 200, double offsetX = 2.0, double offsetY = 2.0, double offsetTheta = M_PI / 4);

	
	int genInitialParticles(int num = 200);

	
	int genParticles();

	
	FieldMarker markerToFieldFrame(FieldMarker marker, Pose2D pose);

	double minDist(FieldMarker marker);

	vector<double> getOffset(FieldMarker marker);

	double residual(vector<FieldMarker> markers_r, Pose2D pose);

	bool isConverged();

	int calcProbs(vector<FieldMarker> markers_r);

	Pose2D finalAdjust(vector<FieldMarker> markers_r, Pose2D pose);

	
	inline double probDesity(double r, double mu, double sigma)
	{
		if (sigma < 1e-5)
			return 0.0;
		return 1 / sqrt(2 * M_PI * sigma * sigma) * exp(-(r - mu) * (r - mu) / (2 * sigma * sigma));
	};
};



class Brain; 
using namespace BT;


void RegisterLocatorNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

class SelfLocate : public SyncActionNode
{
public:
    SelfLocate(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<string>("mode", "enter_field", "must be one of [trust_direction, face_forward, fall_recovery]"),
            InputPort<double>("msecs_interval", 10000, "Prevent overly frequent calibration; if last calibration was within this interval, do not recalibrate."),
        };
    };

private:
    Brain *brain;
};

class SelfLocateEnterField : public SyncActionNode
{
public:
    SelfLocateEnterField(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 1000, "Prevent overly frequent calibration; if last calibration was within this interval, do not recalibrate."),
        };
    };

private:
    Brain *brain;
};

class SelfLocate1M : public SyncActionNode
{
public:
    SelfLocate1M(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 1000, "Prevent overly frequent calibration; if last calibration was within this interval, do not recalibrate."),
            InputPort<double>("max_dist", 2.0, "Only calibrate when marker distance to robot is less than this value (shorter distance yields more accurate measurement)."),
            InputPort<double>("max_drift", 1.0, "After calibration, the displacement from the original position must be less than this value; otherwise calibration is considered failed."),
            InputPort<bool>("validate", true, "After calibration, validate with other markers; require less than locator's max residual"),
        };
    };

private:
    Brain *brain;
};

class SelfLocate2X : public SyncActionNode
{
public:
    SelfLocate2X(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 1000, "Prevent overly frequent calibration; if last calibration was within this interval, do not recalibrate."),
            InputPort<double>("max_dist", 2.0, "Only calibrate when penalty point distance to robot is less than this value (shorter distance yields more accurate measurement)."),
            InputPort<double>("max_drift", 1.0, "After calibration, the displacement from the original position must be less than this value; otherwise calibration is considered failed."),
            InputPort<bool>("validate", true, "After calibration, validate with other markers, requiring less than locator's max residual"),
        };
    };

private:
    Brain *brain;
};

class SelfLocate2T : public SyncActionNode
{
public:
    SelfLocate2T(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 1000, "Prevent overly frequent calibration; if last calibration was within this interval, do not recalibrate."),
            InputPort<double>("max_dist", 2.0, "Only calibrate when both TCross markers are within this distance to the robot (shorter distance yields more accurate measurement)."),
            InputPort<double>("max_drift", 1.0, "After calibration, the displacement from the original position must be less than this value; otherwise calibration is considered failed."),
            InputPort<bool>("validate", true, "After calibration, validate with other markers, requiring less than locator's max residual"),
        };
    };

private:
    Brain *brain;
};

class SelfLocateLT : public SyncActionNode
{
public:
    SelfLocateLT(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 1000, "Prevent overly frequent calibration; if last calibration was within this interval, do not recalibrate."),
            InputPort<double>("max_dist", 2.0, "Only calibrate when penalty point distance to robot is less than this value (shorter distance yields more accurate measurement)."),
            InputPort<double>("max_drift", 1.0, "After calibration, the displacement from the original position must be less than this value; otherwise calibration is considered failed."),
            InputPort<bool>("validate", true, "After calibration, validate with other markers, requiring less than locator's max residual"),
        };
    };

private:
    Brain *brain;
};

class SelfLocatePT : public SyncActionNode
{
public:
    SelfLocatePT(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 1000, "Prevent overly frequent calibration; if last calibration was within this interval, do not recalibrate."),
            InputPort<double>("max_dist", 2.0, "Only calibrate when penalty point distance to robot is less than this value (shorter distance yields more accurate measurement)."),
            InputPort<double>("max_drift", 1.0, "After calibration, the displacement from the original position must be less than this value; otherwise calibration is considered failed."),
            InputPort<bool>("validate", true, "After calibration, validate with other markers, requiring less than locator's max residual"),
        };
    };

private:
    Brain *brain;
};

class SelfLocateBorder : public SyncActionNode
{
public:
    SelfLocateBorder(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 1000, "Prevent overly frequent calibration; if last calibration was within this interval, do not recalibrate."),
            InputPort<double>("max_dist", 2.0, "Only calibrate when border distance to robot is less than this value (shorter distance yields more accurate measurement)."),
            InputPort<double>("max_drift", 1.0, "After calibration, the displacement from the original position must be less than this value; otherwise calibration is considered failed."),
            InputPort<bool>("validate", true, "After calibration, validate with other markers, requiring less than locator's max residual"),
        };
    };

private:
    Brain *brain;
};
