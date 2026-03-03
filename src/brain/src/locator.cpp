#include "locator.h"
#include "utils/math.h"
#include "utils/misc.h"
#include "brain.h"
#include "utils/print.h"
#include "brain_tree.h"

#define REGISTER_LOCATOR_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });

void RegisterLocatorNodes(BT::BehaviorTreeFactory &factory, Brain* brain)
{
    REGISTER_LOCATOR_BUILDER(SelfLocate);
    REGISTER_LOCATOR_BUILDER(SelfLocateEnterField);
    REGISTER_LOCATOR_BUILDER(SelfLocate1M);
    REGISTER_LOCATOR_BUILDER(SelfLocateBorder);
    REGISTER_LOCATOR_BUILDER(SelfLocate2T);
    REGISTER_LOCATOR_BUILDER(SelfLocateLT);
    REGISTER_LOCATOR_BUILDER(SelfLocatePT);
    REGISTER_LOCATOR_BUILDER(SelfLocate2X);
}

void Locator::init(FieldDimensions fd, int minMarkerCntParam, double residualToleranceParam, double muOffestParam)
{
    fieldDimensions = fd;
    calcFieldMarkers(fd);
    minMarkerCnt = minMarkerCntParam;
    residualTolerance = residualToleranceParam;
    muOffset = muOffestParam;
}

void Locator::calcFieldMarkers(FieldDimensions fd)
{
    
    fieldMarkers.push_back(FieldMarker{'X', 0.0, -fd.circleRadius, 0.0});
    fieldMarkers.push_back(FieldMarker{'X', 0.0, fd.circleRadius, 0.0});

    fieldMarkers.push_back(FieldMarker{'P', fd.length / 2 - fd.penaltyDist, 0.0, 0.0});
    fieldMarkers.push_back(FieldMarker{'P', -fd.length / 2 + fd.penaltyDist, 0.0, 0.0});

    fieldMarkers.push_back(FieldMarker{'T', 0.0, fd.width / 2, 0.0});
    fieldMarkers.push_back(FieldMarker{'T', 0.0, -fd.width / 2, 0.0});

    fieldMarkers.push_back(FieldMarker{'L', (fd.length / 2 - fd.penaltyAreaLength), fd.penaltyAreaWidth / 2, 0.0});
    fieldMarkers.push_back(FieldMarker{'L', (fd.length / 2 - fd.penaltyAreaLength), -fd.penaltyAreaWidth / 2, 0.0});
    fieldMarkers.push_back(FieldMarker{'L', -(fd.length / 2 - fd.penaltyAreaLength), fd.penaltyAreaWidth / 2, 0.0});
    fieldMarkers.push_back(FieldMarker{'L', -(fd.length / 2 - fd.penaltyAreaLength), -fd.penaltyAreaWidth / 2, 0.0});
    fieldMarkers.push_back(FieldMarker{'T', fd.length / 2, fd.penaltyAreaWidth / 2, 0.0});
    fieldMarkers.push_back(FieldMarker{'T', fd.length / 2, -fd.penaltyAreaWidth / 2, 0.0});
    fieldMarkers.push_back(FieldMarker{'T', -fd.length / 2, fd.penaltyAreaWidth / 2, 0.0});
    fieldMarkers.push_back(FieldMarker{'T', -fd.length / 2, -fd.penaltyAreaWidth / 2, 0.0});

    fieldMarkers.push_back(FieldMarker{'L', (fd.length / 2 - fd.goalAreaLength), fd.goalAreaWidth / 2, 0.0});
    fieldMarkers.push_back(FieldMarker{'L', (fd.length / 2 - fd.goalAreaLength), -fd.goalAreaWidth / 2, 0.0});
    fieldMarkers.push_back(FieldMarker{'L', -(fd.length / 2 - fd.goalAreaLength), fd.goalAreaWidth / 2, 0.0});
    fieldMarkers.push_back(FieldMarker{'L', -(fd.length / 2 - fd.goalAreaLength), -fd.goalAreaWidth / 2, 0.0});
    fieldMarkers.push_back(FieldMarker{'T', fd.length / 2, fd.goalAreaWidth / 2, 0.0});
    fieldMarkers.push_back(FieldMarker{'T', fd.length / 2, -fd.goalAreaWidth / 2, 0.0});
    fieldMarkers.push_back(FieldMarker{'T', -fd.length / 2, fd.goalAreaWidth / 2, 0.0});
    fieldMarkers.push_back(FieldMarker{'T', -fd.length / 2, -fd.goalAreaWidth / 2, 0.0});

    fieldMarkers.push_back(FieldMarker{'L', fd.length / 2, fd.width / 2, 0.0});
    fieldMarkers.push_back(FieldMarker{'L', fd.length / 2, -fd.width / 2, 0.0});
    fieldMarkers.push_back(FieldMarker{'L', -fd.length / 2, fd.width / 2, 0.0});
    fieldMarkers.push_back(FieldMarker{'L', -fd.length / 2, -fd.width / 2, 0.0});
}

FieldMarker Locator::markerToFieldFrame(FieldMarker marker_r, Pose2D pose_r2f)
{
    auto [x, y, theta] = pose_r2f;

    Eigen::Matrix3d transform;
    transform << cos(theta), -sin(theta), x,
        sin(theta), cos(theta), y,
        0, 0, 1;

    Eigen::Vector3d point_r;
    point_r << marker_r.x, marker_r.y, 1.0;

    auto point_f = transform * point_r;

    return FieldMarker{marker_r.type, point_f.x(), point_f.y(), marker_r.confidence};
}

int Locator::genInitialParticles(int num)
{
    unsigned long long seed = chr::duration_cast<std::chrono::milliseconds>(chr::system_clock::now().time_since_epoch()).count();
    srand(seed);
    hypos.resize(num, 6);
    hypos.leftCols(3) = Eigen::ArrayXXd::Random(num, 3);

    auto [xmin, xmax, ymin, ymax, thetamin, thetamax] = constraints;
    hypos.col(0) = hypos.col(0) * (xmax - xmin) / 2 + (xmin + xmax) / 2;
    hypos.col(1) = hypos.col(1) * (ymax - ymin) / 2 + (ymin + ymax) / 2;
    hypos.col(2) = hypos.col(2) * (thetamax - thetamin) / 2 + (thetamin + thetamax) / 2;

    return 0;
}

int Locator::genParticles()
{
    auto old_hypos = hypos;

    int num = static_cast<int>(hypos.rows() * numShrinkRatio);
    if (num <= 0)
        return 1;
    hypos.resize(num, 6);
    hypos.setZero();
    Eigen::ArrayXd rands = (Eigen::ArrayXd::Random(num) + 1) / 2;

    for (int i = 0; i < rands.size(); i++)
    {
        double rand = rands(i);
        int j;
        for (j = 0; j < old_hypos.rows(); j++)
        {
            if (old_hypos(j, 5) >= rand)
                break;
        }
        hypos.row(i).head(3) = old_hypos.row(j).head(3);
    }

    offsetX *= offsetShrinkRatio;
    offsetY *= offsetShrinkRatio;
    offsetTheta *= offsetShrinkRatio;
    Eigen::ArrayXXd offsets = Eigen::ArrayXXd::Random(num, 3);
    offsets.col(0) *= offsetX;
    offsets.col(1) *= offsetY;
    offsets.col(2) *= offsetTheta;
    hypos.leftCols(3) += offsets;

    hypos.col(0) = hypos.col(0).cwiseMax(constraints.xmin).cwiseMin(constraints.xmax);
    hypos.col(1) = hypos.col(1).cwiseMax(constraints.ymin).cwiseMin(constraints.ymax);
    hypos.col(2) = hypos.col(2).cwiseMax(constraints.thetamin).cwiseMin(constraints.thetamax);

    return 0;
}

double Locator::minDist(FieldMarker marker)
{
    double minDist = std::numeric_limits<double>::infinity();
    double dist;
    for (int i = 0; i < fieldMarkers.size(); i++)
    {
        auto target = fieldMarkers[i];
        if (target.type != marker.type)
        {
            continue;
        }
        dist = sqrt(pow((target.x - marker.x), 2.0) + pow((target.y - marker.y), 2.0));
        if (dist < minDist)
            minDist = dist;
    }
    return minDist;
}

vector<double> Locator::getOffset(FieldMarker marker)
{
    double minDist = std::numeric_limits<double>::infinity();
    FieldMarker nearestTarget{' ', 0.0, 0.0, 0.0};
    double dist;
    for (int i = 0; i < fieldMarkers.size(); i++)
    {
        auto target = fieldMarkers[i];
        if (target.type != marker.type)
        {
            continue;
        }
        dist = sqrt(pow((target.x - marker.x), 2.0) + pow((target.y - marker.y), 2.0));
        if (dist < minDist)
        {
            minDist = dist;
            nearestTarget = target;
        }
    }

    return vector<double>{nearestTarget.x - marker.x, nearestTarget.y - marker.y};
}

double Locator::residual(vector<FieldMarker> markers_r, Pose2D pose)
{
    double res = 0;

    for (int i = 0; i < markers_r.size(); i++)
    {
        auto marker_r = markers_r[i];
        double dist = max(norm(marker_r.x, marker_r.y), 0.1);
        auto marker_f = markerToFieldFrame(marker_r, pose);
        double conf = max(marker_r.confidence, 0.1);
        res += minDist(marker_f) / dist * 3; // weighted
    }

    return res;
}

Pose2D Locator::finalAdjust(vector<FieldMarker> markers_r, Pose2D pose)
{
    if (markers_r.size() == 0)
        return Pose2D{0, 0, 0};

    double dx = 0;
    double dy = 0;
    for (int i = 0; i < markers_r.size(); i++)
    {
        auto marker_r = markers_r[i];
        auto marker_f = markerToFieldFrame(marker_r, pose);
        auto offset = getOffset(marker_f);
        dx += offset[0];
        dy += offset[1];
    }
    dx /= markers_r.size();
    dy /= markers_r.size();

    return Pose2D{pose.x + dx, pose.y + dy, pose.theta};
}

int Locator::calcProbs(vector<FieldMarker> markers_r)
{
    int rows = hypos.rows();
    if (rows <= 1)
        return 1; 

    for (int i = 0; i < rows; i++)
    {
        Pose2D pose{hypos(i, 0), hypos(i, 1), hypos(i, 2)};
        double res = residual(markers_r, pose);
        hypos(i, 3) = res;

        if (res < bestResidual)
        {
            bestResidual = res;
            bestPose = pose;
        }
    }

    double mean = hypos.col(3).mean();
    double sqSum = ((hypos.col(3) - mean).square().sum());
    double sigma = std::sqrt(sqSum / (rows - 1));
    double mu = hypos.col(3).minCoeff() - muOffset * sigma;

    for (int i = 0; i < rows; i++)
    {
        hypos(i, 4) = probDesity(hypos(i, 3), mu, sigma);
    }

    double probSum = hypos.col(4).sum();
    if (fabs(probSum) < 1e-5)
        return 1;

    hypos.col(4) = hypos.col(4) / probSum;

    double acc = 0;
    for (int i = 0; i < rows; i++)
    {
        acc += hypos(i, 4);
        hypos(i, 5) = acc;
    }

    return 0;
}

bool Locator::isConverged()
{
    return (
        (hypos.col(0).maxCoeff() - hypos.col(0).minCoeff() < convergeTolerance)
        && (hypos.col(1).maxCoeff() - hypos.col(1).minCoeff() < convergeTolerance)
        && (hypos.col(2).maxCoeff() - hypos.col(2).minCoeff() < convergeTolerance)
    );
}

LocateResult Locator::locateRobot(vector<FieldMarker> markers_r, PoseBox2D constraintsParam, int numParticles, double offsetXParam, double offsetYParam, double offsetThetaParam)
{
    auto start_time = chr::high_resolution_clock::now();
    LocateResult res;
    double bigEnoughNum = 1e6;
    res.residual = bigEnoughNum;
    bestResidual = bigEnoughNum;
    bestPose = Pose2D{0.0, 0.0, 0.0};

    if (markers_r.size() < minMarkerCnt)
    {
        res.success = false;
        res.code = 4;
        res.msecs = msecsSince(start_time);
        return res;
    }

    constraints = constraintsParam;
    offsetX = offsetXParam;
    offsetY = offsetYParam;
    offsetTheta = offsetThetaParam;

    genInitialParticles(numParticles);
    if (calcProbs(markers_r))
    {
        res.success = false;
        res.code = 5;
        res.msecs = msecsSince(start_time);
        return res;
    }

    for (int i = 0; i < maxIteration; i++)
    {
        res.residual = bestResidual / markers_r.size();
        if (isConverged())
        {
            if (res.residual > residualTolerance)
            {
                res.success = false;
                res.code = 2;
                res.msecs = msecsSince(start_time);
                return res;
            }
            res.success = true;
            res.code = 0;
            res.pose = bestPose;
            res.pose.theta = toPInPI(res.pose.theta);
            res.msecs = msecsSince(start_time);
            return res;
        }

        if (genParticles())
        {
            res.success = false;
            res.code = 1;
            res.msecs = msecsSince(start_time);
            return res;
        }

        if (calcProbs(markers_r))
        {
            res.success = false;
            res.code = 5;
            res.msecs = msecsSince(start_time);
            return res;
        }
    }

    res.success = false;
    res.code = 3;
    res.msecs = msecsSince(start_time);
    return res;
}


NodeStatus SelfLocate::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    if (brain->msecsSince(brain->data->lastSuccessfulLocalizeTime) < interval) return NodeStatus::SUCCESS;

    string mode = getInput<string>("mode").value();
    double xMin = 0.0, xMax = 0.0, yMin = 0.0, yMax = 0.0, thetaMin = 0.0, thetaMax = 0.0; 
    auto markers = brain->data->getMarkersForLocator();

    if (mode == "face_forward")
    {
        xMin = -brain->config->fieldDimensions.length / 2;
        xMax = brain->config->fieldDimensions.length / 2;
        yMin = -brain->config->fieldDimensions.width / 2;
        yMax = brain->config->fieldDimensions.width / 2;
        thetaMin = -M_PI / 4;
        thetaMax = M_PI / 4;
    }
    else if (mode == "trust_direction")
    {
        int msec = static_cast<int>(brain->msecsSince(brain->data->lastSuccessfulLocalizeTime));
        double maxDriftSpeed = 0.1;                      
        double maxDrift = msec / 1000.0 * maxDriftSpeed; 

        xMin = max(-brain->config->fieldDimensions.length / 2 - 2, brain->data->robotPoseToField.x - maxDrift);
        xMax = min(brain->config->fieldDimensions.length / 2 + 2, brain->data->robotPoseToField.x + maxDrift);
        yMin = max(-brain->config->fieldDimensions.width / 2 - 2, brain->data->robotPoseToField.y - maxDrift);
        yMax = min(brain->config->fieldDimensions.width / 2 + 2, brain->data->robotPoseToField.y + maxDrift);
        thetaMin = brain->data->robotPoseToField.theta - M_PI / 180;
        thetaMax = brain->data->robotPoseToField.theta + M_PI / 180;
    }
    else if (mode == "fall_recovery")
    {
        int msec = static_cast<int>(brain->msecsSince(brain->data->lastSuccessfulLocalizeTime));
        double maxDriftSpeed = 0.1;                      
        double maxDrift = msec / 1000.0 * maxDriftSpeed; 

        xMin = -brain->config->fieldDimensions.length / 2 - 2;
        xMax = brain->config->fieldDimensions.length / 2 + 2;
        yMin = -brain->config->fieldDimensions.width / 2 - 2;
        yMax = brain->config->fieldDimensions.width / 2 + 2;
        thetaMin = brain->data->robotPoseToField.theta - M_PI / 180;
        thetaMax = brain->data->robotPoseToField.theta + M_PI / 180;
    }

    
    
    PoseBox2D constraints{xMin, xMax, yMin, yMax, thetaMin, thetaMax};
    double residual;
    auto res = brain->locator->locateRobot(markers, constraints);

    string mstring = "";
    for (int i = 0; i < markers.size(); i++) {
        auto m = markers[i];
        mstring += format("type: %c  x: %.1f y: %.1f", m.type, m.x, m.y).c_str();
    }
    RCLCPP_INFO(
        brain->get_logger(),
        format2cstr(
            "%s success: %d  residual: %.2f  marker.size: %d  minMarkerCnt: %d  resTolerance: %.2f marker: %s",
            "self_locate",
            res.success,
            res.residual,
            markers.size(),
            brain->locator->minMarkerCnt,
            brain->locator->residualTolerance,
            mstring.c_str()
        )
    );
    
    if (!res.success)
        return NodeStatus::SUCCESS; 

    brain->calibrateOdom(res.pose.x, res.pose.y, res.pose.theta);
    brain->tree->setEntry<bool>("odom_calibrated", true);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    prtDebug("Locate Success: " + to_string(res.pose.x) + " " + to_string(res.pose.y) + " " + to_string(rad2deg(res.pose.theta)) + " Dur: " + to_string(res.msecs));

    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocateEnterField::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    if (brain->msecsSince(brain->data->lastSuccessfulLocalizeTime) < interval) return NodeStatus::SUCCESS;

    auto markers = brain->data->getMarkersForLocator();
    auto fd = brain->config->fieldDimensions;
    PoseBox2D cEnterLeft = {-fd.length / 2, -fd.circleRadius, fd.width / 2, fd.width / 2 + 1, -M_PI / 2 - M_PI / 6, -M_PI / 2 + M_PI / 6};
    PoseBox2D cEnterRight = {-fd.length / 2, -fd.circleRadius, -fd.width / 2 - 1, -fd.width / 2, M_PI / 2 - M_PI / 6, M_PI / 2 + M_PI / 6};


    auto resLeft = brain->locator->locateRobot(markers, cEnterLeft);
    auto resRight = brain->locator->locateRobot(markers, cEnterRight);
    LocateResult res;

    static string lastReport = "";
    string report = lastReport;
    if (resLeft.success && !resRight.success) {
        res = resLeft;
        report = "Entering Left";
    }
    else if (!resLeft.success && resRight.success) {
        res = resRight;
        report = "Entering Right";
    }
    else if (resLeft.success && resRight.success) {
        if (resLeft.residual < resRight.residual) {
            res = resLeft;
            report = "Entering Left";
        }
        else {
            res = resRight;
            report = "Entering Right";
        }
    } else res = resLeft;

    if (report != lastReport) {
        lastReport = report;
    }

    RCLCPP_INFO(
        brain->get_logger(),
        format2cstr(
            "%s %s left success: %d  left residual: %.2f  right success %d  right residual %.2f resTolerance: %.2f markers: %d minMarkerCnt: %d ",
            "SelfLocateEnterField",
            report.c_str(),
            resLeft.success, 
                resLeft.residual,
                resRight.success,
                resRight.residual,
                brain->locator->residualTolerance,
                markers.size(),
                brain->locator->minMarkerCnt
            ),
            res.success
        );

    if (!res.success) return NodeStatus::SUCCESS; 


    // else, success.
    brain->calibrateOdom(res.pose.x, res.pose.y, res.pose.theta);
    brain->tree->setEntry<bool>("odom_calibrated", true);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    prtDebug("Locate Success: " + to_string(res.pose.x) + " " + to_string(res.pose.y) + " " + to_string(rad2deg(res.pose.theta)) + " Dur: " + to_string(res.msecs));

    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocate1M::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; 
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    string logPathS = "/locate/1m/success";
    string logPathF = "/locate/1m/fail";

    
    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, msecs(%.1f).c_str() < interval(%.1f)", logPathF.c_str(), msecs, interval)
        );
        return NodeStatus::SUCCESS;
    }

    int markerIndex = -1;
    GameObject marker;
    MapMarking mapMarker; 
    double minDist = 100;
    auto markings = brain->data->getMarkings();
    for (int i = 0; i < markings.size(); i++) {
        auto m = markings[i];

        
        if (m.name == "LOLG" || m.name == "LORG" || m.name == "LSLG" || m.name == "LSRG") continue; 

        if (m.range < minDist) {
            minDist = m.range;
            markerIndex = i;
            marker = m;
        }
    }
    
    if (
        markerIndex < 0 || markerIndex >= markings.size()
        || marker.id < 0 || marker.id >= brain->config->mapMarkings.size()
    ) {
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, No markings Found. Or marker id invalid.", logPathF.c_str())
        );
        return NodeStatus::SUCCESS;
    }
    mapMarker = brain->config->mapMarkings[marker.id];

    if (marker.range > maxDist) {
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, min marker Dist(%.2f).c_str() > maxDist(%.2f)", logPathF.c_str(), marker.range, maxDist)
        );
        return NodeStatus::SUCCESS;
    }

    if (!brain->isBoundingBoxInCenter(marker.boundingBox)) {
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, boundingbox is not in the center area", logPathF.c_str())
        );
        return NodeStatus::SUCCESS;
    }

    double dx, dy; 
    dx = mapMarker.x - marker.posToField.x;
    dy = mapMarker.y - marker.posToField.y;

    double drift = norm(dx, dy);
    if (drift > maxDrift) {
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, drift(%.2f).c_str() > maxDrift(%.2f)", logPathF.c_str(), drift, maxDrift)
        );
        return NodeStatus::SUCCESS;
    }
    
    
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) 
        { 
            RCLCPP_INFO(
                brain->get_logger(),
                format2cstr("%s, Failed, validation residual(%.2f).c_str() > tolerance(%.2f)", logPathF.c_str(), residual, brain->locator->residualTolerance)
            );
            return NodeStatus::SUCCESS;
        }
    }

    RCLCPP_INFO(
        brain->get_logger(),
        format2cstr("%s, Success. Drift = %.2f", logPathS.c_str(), drift)
    );
    brain->calibrateOdom(hypoPose.x, hypoPose.y, hypoPose.theta);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocate2X::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; 
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    string logPathS = "/locate/2x/success";
    string logPathF = "/locate/2x/fail";

    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, msecs(%.1f).c_str() < interval(%.1f)", logPathF.c_str(), msecs, interval)
        );
        return NodeStatus::SUCCESS;
    }

    auto points = brain->data->getMarkingsByType({"XCross"});
    if (points.size() != 2) {
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, point cnt(%d).c_str() != 2", logPathF.c_str(), points.size())
        );
        return NodeStatus::SUCCESS;
    }

    auto p0 = points[0]; auto p1 = points[1];
    
    if (p0.range > maxDist || p1.range > maxDist) 
    { 
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, p0 range (%.2f).c_str() or p1 range (%.2f) > maxDist(%.2f)", logPathF.c_str(), p0.range, p1.range, maxDist)
        );
        return NodeStatus::SUCCESS;
    }

    double xDist = fabs(p0.posToField.x - p1.posToField.x);
    if (xDist > 0.5) 
    { 
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, xDist(%.2f).c_str() > maxDist(%.2f)", logPathF.c_str(), xDist, 0.5)
        );
        return NodeStatus::SUCCESS;
    }

    double yDist = fabs(p0.posToField.y - p1.posToField.y);
    double mapYDist = brain->config->fieldDimensions.circleRadius * 2.0;
    if (fabs(yDist - mapYDist) > 0.5) { 
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, yDist(%.2f).c_str() too far (%.2f) from mapYDist(%.2f)", logPathF.c_str(), yDist, 0.5, mapYDist)
        );
        return NodeStatus::SUCCESS;
    }

    
    double dx = - (p0.posToField.x + p1.posToField.x) / 2.0;
    double dy = - (p1.posToField.y + p1.posToField.y) / 2.0;
    double drift = norm(dx, dy);

    if (drift > maxDrift) { 
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, dirft(%.2f).c_str() > maxDrift(%.2f)", logPathF.c_str(), drift, maxDrift)
        );
        return NodeStatus::SUCCESS;
    }

    
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) 
        { 
            RCLCPP_INFO(
                brain->get_logger(),
                format2cstr("%s, Failed, validation residual(%.2f).c_str() > tolerance(%.2f)", logPathF.c_str(), residual, brain->locator->residualTolerance)
            );
            return NodeStatus::SUCCESS;
        }
    }

    RCLCPP_INFO(
        brain->get_logger(),
        format2cstr("%s, Success. Dist = %.2f", logPathS.c_str(), drift)
    );
    brain->calibrateOdom(hypoPose.x, hypoPose.y, hypoPose.theta);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocate2T::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; 
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    string logPathS = "/locate/2t/success";
    string logPathF = "/locate/2t/fail";

    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, msecs(%.1f).c_str() < interval(%.1f)", logPathF.c_str(), msecs, interval)
        );
        return NodeStatus::SUCCESS;
    }

    auto markers = brain->data->getMarkingsByType({"TCross"});
    GameObject m1, m2;
    bool found = false;
    auto fd = brain->config->fieldDimensions;
    for (int i = 0; i < markers.size(); i++) {
        m1 = markers[i];
        
        if (m1.range > maxDist) continue; 

        for (int j = i + 1; j < markers.size(); j++) {
            m2 = markers[j];

            if (m2.range > maxDist) continue; 

            if (
                fabs(m1.posToField.x - m2.posToField.x) < 0.3
                && fabs(fabs(m1.posToField.y - m2.posToField.y) - fabs(fd.goalAreaWidth - fd.penaltyAreaWidth)/2.0)< 0.3
            ) {
                found = true;
                break;
            }
        }
        if (found) break;
    }


    if (!found) {
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, No pattern within maxDist(%.2f).c_str() Found", logPathF.c_str(), maxDist)
        );
        return NodeStatus::SUCCESS;
    }

    Point2D pos_o = { 
        (m1.posToField.x + m2.posToField.x)/2,
        (m1.posToField.y + m2.posToField.y)/2
    };
    Point2D pos_m; 

    vector<double> halfs = {-1, 1};
    vector<double> sides = {-1, 1};
    bool matched = false;
    for (auto half: halfs) {
        for (auto side: sides) {
            pos_m = {
                half * (fd.length / 2.0), 
                side * (fd.penaltyAreaWidth + fd.goalAreaWidth) / 4.0
            };
            double dist = norm(pos_o.x - pos_m.x, pos_o.y - pos_m.y);
            if (dist < maxDrift) {
                matched = true;
                break;
            }
        }
        if (matched) break;
    }

    if (!matched) {
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, can not match to any map positions within maxDrift(%.2f).c_str()", logPathF.c_str(), maxDrift)
        );
        return NodeStatus::SUCCESS;
    }

    double dx = pos_m.x - pos_o.x;
    double dy = pos_m.y - pos_o.y;
    double drift = norm(dx, dy);

    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) 
        { 
            RCLCPP_INFO(
                brain->get_logger(),
                format2cstr("%s, Failed, validation residual(%.2f).c_str() > tolerance(%.2f)", logPathF.c_str(), residual, brain->locator->residualTolerance)
            );
            return NodeStatus::SUCCESS;
        }
    }

    RCLCPP_INFO(
        brain->get_logger(),
        format2cstr("%s, Success. Dist = %.2f", logPathS.c_str(), drift)
    );
    brain->calibrateOdom(hypoPose.x, hypoPose.y, hypoPose.theta);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocateLT::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; 
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    string logPathS = "/locate/lt/success";
    string logPathF = "/locate/lt/fail";

    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, msecs(%.1f).c_str() < interval(%.1f)", logPathF.c_str(), msecs, interval)
        );
        return NodeStatus::SUCCESS;
    }

    auto tMarkers = brain->data->getMarkingsByType({"TCross"});
    auto lMarkers = brain->data->getMarkingsByType({"LCross"});

    GameObject t, l;
    bool found = false;
    auto fd = brain->config->fieldDimensions;
    for (int i = 0; i < tMarkers.size(); i++) {
        t = tMarkers[i];
        
        if (t.range > maxDist) continue; 

        for (int j = i + 1; j < lMarkers.size(); j++) {
            l = lMarkers[j];

            if (l.range > maxDist) continue;

            if (
                fabs(t.posToField.y - l.posToField.y) < 0.3
                && fabs(fabs(t.posToField.x - l.posToField.x) - fd.goalAreaLength)< 0.3
            ) {
                found = true;
                break;
            }
        }

        if (found) break;
    }


    if (!found) {
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, No pattern within maxDist(%.2f).c_str() Found", logPathF.c_str(), maxDist)
        );
        return NodeStatus::SUCCESS;
    }

    Point2D pos_o = { 
        (t.posToField.x + l.posToField.x)/2,
        (t.posToField.y + l.posToField.y)/2
    };
    Point2D pos_m; 

    vector<double> halfs = {-1, 1};
    vector<double> sides = {-1, 1};
    bool matched = false;
    for (auto half: halfs) {
        for (auto side: sides) {
            pos_m = {
                half * (fd.length / 2.0 - fd.goalAreaLength / 2.0), 
                side * (fd.goalAreaWidth / 2.0)
            };
            double dist = norm(pos_o.x - pos_m.x, pos_o.y - pos_m.y);
            if (dist < maxDrift) {
                matched = true;
                break;
            }
        }
        if (matched) break;
    }
    if (!matched) {
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, can not match to any map positions within maxDrift(%.2f).c_str()", logPathF.c_str(), maxDrift)
        );
        return NodeStatus::SUCCESS;
    }

    double dx = pos_m.x - pos_o.x;
    double dy = pos_m.y - pos_o.y;
    double drift = norm(dx, dy);

    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { 
            RCLCPP_INFO(
                brain->get_logger(),
                format2cstr("%s, Failed, validation residual(%.2f).c_str() > tolerance(%.2f)", logPathF.c_str(), residual, brain->locator->residualTolerance)
            );
            return NodeStatus::SUCCESS;
        }
    }

    RCLCPP_INFO(
        brain->get_logger(),
        format2cstr("%s, Success. Dist = %.2f", logPathS.c_str(), drift)
    );
    brain->calibrateOdom(hypoPose.x, hypoPose.y, hypoPose.theta);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocatePT::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; 
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    string logPathS = "/locate/pt/success";
    string logPathF = "/locate/pt/fail";

    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, msecs(%.1f).c_str() < interval(%.1f)", logPathF.c_str(), msecs, interval)
        );
        return NodeStatus::SUCCESS;
    }

    auto posts = brain->data->getGoalposts();
    auto tMarkers = brain->data->getMarkingsByType({"TCross"});
    
    GameObject p, t;
    bool found = false;
    auto fd = brain->config->fieldDimensions;
    for (int i = 0; i < posts.size(); i++) {
        p = posts[i];
        if (p.range > maxDist) continue;

        for (int j = i + 1; j < tMarkers.size(); j++) {
            t = tMarkers[j];
            if (t.range > maxDist) continue;
            if (
                fabs(t.posToField.x - p.posToField.x) < 0.5
                && fabs(fabs(t.posToField.x - p.posToField.x) - fabs(fd.goalAreaWidth - fd.goalWidth) / 2.0) < 0.3
            ) {
                found = true;
                break;
            }
        }
        
        if (found) break;
    }


    if (!found) {
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, No pattern within maxDist(%.2f).c_str() Found", logPathF.c_str(), maxDist)
        );
        return NodeStatus::SUCCESS;
    }

    Point2D pos_o = { 
        t.posToField.x,
        t.posToField.y
    };
    Point2D pos_m; 

    vector<double> halfs = {-1, 1};
    vector<double> sides = {-1, 1};
    bool matched = false;
    for (auto half: halfs) {
        for (auto side: sides) {
            pos_m = {
                half * (fd.length), 
                side * (fd.goalAreaWidth / 2.0)
            };
            double dist = norm(pos_o.x - pos_m.x, pos_o.y - pos_m.y);
            if (dist < maxDrift) {
                matched = true;
                break;
            }
        }
        if (matched) break;
    }
    if (!matched) {
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, can not match to any map positions within maxDrift(%.2f).c_str()", logPathF.c_str(), maxDrift)
        );
        return NodeStatus::SUCCESS;
    }

    double dx = pos_m.x - pos_o.x;
    double dy = pos_m.y - pos_o.y;
    double drift = norm(dx, dy);

    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { 

            RCLCPP_INFO(
                brain->get_logger(),
                format2cstr("%s, Failed, validation residual(%.2f).c_str() > tolerance(%.2f)", logPathF.c_str(), residual, brain->locator->residualTolerance)
            );
            return NodeStatus::SUCCESS;
        }
    }

    
    RCLCPP_INFO(
        brain->get_logger(),
        format2cstr("%s, Success. Dist = %.2f", logPathS.c_str(), drift)
    );
    brain->calibrateOdom(hypoPose.x, hypoPose.y, hypoPose.theta);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocateBorder::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; 
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    string logPathS = "/locate/border/success";
    string logPathF = "/locate/border/fail";

    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, msecs(%.1f).c_str() < interval(%.1f)", logPathF.c_str(), msecs, interval)
        );
        return NodeStatus::SUCCESS;
    }

    
    bool touchLineFound = false;
    FieldLine touchLine;
    double bestConfidenceTouchline = 0.0;
    bool goalLineFound = false;
    FieldLine goalLine;
    double bestConfidenceGoalline = 0.0;
    bool middleLineFound = false;
    FieldLine middleLine;
    double bestConfidenceMiddleLine = 0.0;

    auto fieldLines = brain->data->getFieldLines();
    for (int i = 0; i < fieldLines.size(); i++) {
        auto line = fieldLines[i];
        if (line.type != LineType::TouchLine && line.type != LineType::GoalLine && line.type != LineType::MiddleLine) continue;
        if (line.confidence < 0.8) continue;
        
        double dist = pointMinDistToLine(
            Point2D({brain->data->robotPoseToField.x, brain->data->robotPoseToField.y}), 
            line.posToField
        );
        if (dist > maxDist) continue;

        if (line.type == LineType::TouchLine) {
           if (line.confidence > bestConfidenceTouchline) {
               bestConfidenceTouchline = line.confidence;
               touchLine = line;
               touchLineFound = true;
           }
        } else if (line.type == LineType::GoalLine) {
            if (line.confidence > bestConfidenceGoalline) {
                bestConfidenceGoalline = line.confidence;
                goalLine = line;
                goalLineFound = true;
            }
        } else if (line.type == LineType::MiddleLine) {
            if (line.confidence > bestConfidenceMiddleLine) {
                bestConfidenceMiddleLine = line.confidence;
                middleLine = line;
                middleLineFound = true;
            }
        }
    }

    double dx = 0; 
    double dy = 0; 
    auto fd = brain->config->fieldDimensions;
    if (touchLineFound) {
       double y_m = touchLine.side == LineSide::Left ? fd.width / 2.0 : - fd.width / 2.0;
       double perpDist = pointPerpDistToLine(
           Point2D({brain->data->robotPoseToField.x, brain->data->robotPoseToField.y}),
           touchLine.posToField
       );
       double y_o = touchLine.side == LineSide::Left ? 
           brain->data->robotPoseToField.y - perpDist :
           brain->data->robotPoseToField.y + perpDist;
       dy = y_m - y_o;
    }
    if (goalLineFound) {
        double x_m = goalLine.half == LineHalf::Opponent ? fd.length / 2.0: - fd.length / 2.0;
        double perpDist = pointPerpDistToLine(
            Point2D({brain->data->robotPoseToField.x, brain->data->robotPoseToField.y}),
            goalLine.posToField
        );
        double x_o = goalLine.half == LineHalf::Opponent?
            brain->data->robotPoseToField.x - perpDist :
            brain->data->robotPoseToField.x + perpDist;
        dx = x_m - x_o;
    } else if (middleLineFound) {
        double x_m = 0;
        auto linePos = middleLine.posToField;
        auto robotPose = brain->data->robotPoseToField;
        vector<double> pointA(2);
        vector<double> pointB(2);
        vector<double> pointR = {robotPose.x, robotPose.y};

        if (linePos.y0 > linePos.y1) {
            pointA = {linePos.x0, linePos.y0};
            pointB = {linePos.x1, linePos.y1};
        } else {
            pointA = {linePos.x1, linePos.y1};
            pointB = {linePos.x0, linePos.y0};
        }

        vector<double> vl = {pointB[0] - pointA[0], pointB[1] - pointA[1]};
        vector<double> vr = {pointR[0] - pointA[0], pointR[1] - pointA[1]};

        double normvl = norm(vl);
        double normvr = norm(vr);
        if (normvl < 1e-3 || normvr < 1e-3) {
            dx = 10000; 
        } else {
            double dist = crossProduct(vr, vl) / normvl;
            double x_o = robotPose.x + dist;
            dx = x_m - x_o;
        }
    }

    
    if ((!touchLineFound && !goalLineFound && !middleLineFound)) {
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, No touchline or goalline or middleLine found.", logPathF.c_str())
        );
        return NodeStatus::SUCCESS;
    }

    double drift = norm(dx, dy);
    if (drift > maxDrift) {
        RCLCPP_INFO(
            brain->get_logger(),
            format2cstr("%s, Failed, drift(%.2f).c_str() > maxDrift(%.2f)", logPathF.c_str(), drift, maxDrift)
        );
        return NodeStatus::SUCCESS;
    }
    
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { 
            RCLCPP_INFO(
                brain->get_logger(),
                format2cstr("%s, Failed, validation residual(%.2f).c_str() > tolerance(%.2f)", logPathF.c_str(), residual, brain->locator->residualTolerance)
            );
            return NodeStatus::SUCCESS;
        }
    }

    RCLCPP_INFO(
        brain->get_logger(),
        format2cstr("%s, Success. Drift = %.2f", logPathS.c_str(), drift)
    );
    string label = "";
    if (touchLineFound) label += "TouchLine";
    if (touchLineFound && (goalLineFound || middleLineFound)) label += " ";
    if (goalLineFound) label += "GoalLine";
    if (middleLineFound) label += "MiddleLine";

    brain->calibrateOdom(hypoPose.x, hypoPose.y, hypoPose.theta);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}