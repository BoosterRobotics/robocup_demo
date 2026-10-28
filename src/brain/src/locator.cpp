#include "locator.h"
#include "utils/math.h"
#include "utils/misc.h"

void Locator::init(FieldDimensions fd, int minMarkerCntParam, double residualToleranceParam, double muOffestParam, bool enableLogParam, string logIPParam)
{
    fieldDimensions = fd;
    calcFieldMarkers(fd);
    minMarkerCnt = minMarkerCntParam;
    residualTolerance = residualToleranceParam;
    muOffset = muOffestParam;
    enableLog = enableLogParam;
    logIP = logIPParam;
    if (enableLog) {
        auto connectError = log.connect(logIP);
        if (connectError.is_err()) prtErr(format("Rerun log connect Error: %s", connectError.description.c_str()));
        auto saveError = log.save("/home/booster/log.rrd");
        if (saveError.is_err()) prtErr(format("Rerun log save Error: %s", saveError.description.c_str()));
    }
    
}

void Locator::calcFieldMarkers(FieldDimensions fd)
{
    // 中线上的 X 标志
    fieldMarkers.push_back(FieldMarker{'X', 0.0, -fd.circleRadius});
    fieldMarkers.push_back(FieldMarker{'X', 0.0, fd.circleRadius});

    // 罚球点
    fieldMarkers.push_back(FieldMarker{'P', fd.length / 2 - fd.penaltyDist, 0.0});
    fieldMarkers.push_back(FieldMarker{'P', -fd.length / 2 + fd.penaltyDist, 0.0});

    // 边线中心
    fieldMarkers.push_back(FieldMarker{'T', 0.0, fd.width / 2});
    fieldMarkers.push_back(FieldMarker{'T', 0.0, -fd.width / 2});

    // 禁区
    fieldMarkers.push_back(FieldMarker{'L', (fd.length / 2 - fd.penaltyAreaLength), fd.penaltyAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'L', (fd.length / 2 - fd.penaltyAreaLength), -fd.penaltyAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'L', -(fd.length / 2 - fd.penaltyAreaLength), fd.penaltyAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'L', -(fd.length / 2 - fd.penaltyAreaLength), -fd.penaltyAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'T', fd.length / 2, fd.penaltyAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'T', fd.length / 2, -fd.penaltyAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'T', -fd.length / 2, fd.penaltyAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'T', -fd.length / 2, -fd.penaltyAreaWidth / 2});

    // 球门区
    fieldMarkers.push_back(FieldMarker{'L', (fd.length / 2 - fd.goalAreaLength), fd.goalAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'L', (fd.length / 2 - fd.goalAreaLength), -fd.goalAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'L', -(fd.length / 2 - fd.goalAreaLength), fd.goalAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'L', -(fd.length / 2 - fd.goalAreaLength), -fd.goalAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'T', fd.length / 2, fd.goalAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'T', fd.length / 2, -fd.goalAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'T', -fd.length / 2, fd.goalAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'T', -fd.length / 2, -fd.goalAreaWidth / 2});

    // 场地四角
    fieldMarkers.push_back(FieldMarker{'L', fd.length / 2, fd.width / 2});
    fieldMarkers.push_back(FieldMarker{'L', fd.length / 2, -fd.width / 2});
    fieldMarkers.push_back(FieldMarker{'L', -fd.length / 2, fd.width / 2});
    fieldMarkers.push_back(FieldMarker{'L', -fd.length / 2, -fd.width / 2});
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

    return FieldMarker{marker_r.type, point_f.x(), point_f.y()};
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

    logParticles();

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

    // 根据概率决定新 particle 采样的基准
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

    // 为基准加入噪音
    offsetX *= offsetShrinkRatio;
    offsetY *= offsetShrinkRatio;
    offsetTheta *= offsetShrinkRatio;
    Eigen::ArrayXXd offsets = Eigen::ArrayXXd::Random(num, 3);
    offsets.col(0) *= offsetX;
    offsets.col(1) *= offsetY;
    offsets.col(2) *= offsetTheta;
    hypos.leftCols(3) += offsets;

    // 限制在 constraints 之内
    hypos.col(0) = hypos.col(0).cwiseMax(constraints.xmin).cwiseMin(constraints.xmax);
    hypos.col(1) = hypos.col(1).cwiseMax(constraints.ymin).cwiseMin(constraints.ymax);
    hypos.col(2) = hypos.col(2).cwiseMax(constraints.thetamin).cwiseMin(constraints.thetamax);

    logParticles();

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
        // else 类型相同
        dist = sqrt(pow((target.x - marker.x), 2.0) + pow((target.y - marker.y), 2.0));
        if (dist < minDist)
            minDist = dist;
    }
    return minDist;
}

vector<double> Locator::getOffset(FieldMarker marker)
{
    double minDist = std::numeric_limits<double>::infinity();
    FieldMarker nearestTarget;
    double dist;
    for (int i = 0; i < fieldMarkers.size(); i++)
    {
        auto target = fieldMarkers[i];
        if (target.type != marker.type)
        {
            continue;
        }
        // else 类型相同
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
        res += minDist(marker_f) / dist * 3; // 加权
        // res += minDist(marker_f) * conf / 100.0 / dist * 3; // 加权 & 考虑 confidence
        // res += minDist(marker_f); // 不加权
    }

    return res;
}

Pose2D Locator::finalAdjust(vector<FieldMarker> markers_r, Pose2D pose)
{
    if (markers_r.size() == 0)
        return Pose2D{0, 0, 0};

    double dx = 0;
    double dy = 0;
    double dtheta = 0;
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
    if (rows < 1)
        return 1; // 防止出现无法计算标准差的情况

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

    double mean = hypos.col(3).mean();                      // 计算平均值
    double sqSum = ((hypos.col(3) - mean).square().sum());  // 计算平方和
    double sigma = std::sqrt(sqSum / (rows - 1));           // 计算标准差
    double mu = hypos.col(3).minCoeff() - muOffset * sigma; // 概率密度分布假设的均值

    for (int i = 0; i < rows; i++)
    {
        hypos(i, 4) = probDesity(hypos(i, 3), mu, sigma);
    }

    double probSum = hypos.col(4).sum();
    if (fabs(probSum) < 1e-5)
        return 1; // 所有的概率都接近 0

    hypos.col(4) = hypos.col(4) / probSum; // 归一化

    // 计算累计概率(从行0到本行的概率之和), 以方便下一次采样
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
        (hypos.col(0).maxCoeff() - hypos.col(0).minCoeff() < convergeTolerance)    // x
        && (hypos.col(1).maxCoeff() - hypos.col(1).minCoeff() < convergeTolerance) // y
        && (hypos.col(2).maxCoeff() - hypos.col(2).minCoeff() < convergeTolerance) // theta TODO: 考虑为 theta
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
    { // Marker 的数量不够
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
    { // 所有概率均过低
        res.success = false;
        res.code = 5;
        res.msecs = msecsSince(start_time);
        return res;
    }

    // else
    for (int i = 0; i < maxIteration; i++)
    {
        res.residual = bestResidual / markers_r.size();
        if (isConverged())
        {
            // 检查残差是否合理
            if (res.residual > residualTolerance)
            { // 收敛后的残差过大
                res.success = false;
                res.code = 2;
                res.msecs = msecsSince(start_time);
                return res;
            }
            // else 残差合理, 定位成功
            // pose = finalAdjust(markers_r, bestPose);
            res.success = true;
            res.code = 0;
            res.pose = bestPose;
            res.pose.theta = toPInPI(res.pose.theta);
            res.msecs = msecsSince(start_time);
            return res;
        }

        // 未收敛
        if (genParticles())
        { // 生成粒子失败
            res.success = false;
            res.code = 1;
            res.msecs = msecsSince(start_time);
            return res;
        }

        if (calcProbs(markers_r))
        { // 所有概率均过低
            res.success = false;
            res.code = 5;
            res.msecs = msecsSince(start_time);
            return res;
        }
    }

    // 达到最大迭代次数, 未收敛
    res.success = false;
    res.code = 3;
    res.msecs = msecsSince(start_time);
    return res; // 未收敛
}

void Locator::logParticles()
{
    vector<rerun::Position2D> points;
    for (int i = 0; i < hypos.rows(); i++)
    {
        auto hypo = hypos.row(i);
        points.push_back(rerun::Position2D{static_cast<float>(hypo(0)), static_cast<float>(hypo(1))});
    }
    log.log(
        "field/hypos",
        rerun::Points2D(points).with_draw_order(20.0).with_colors({rerun::Color{255, 0, 0, 255}}).with_radii({0.05}).with_draw_order(20));
}
