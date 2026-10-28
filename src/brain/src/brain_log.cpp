#include "brain_log.h"
#include "brain.h"
#include "utils/math.h"
#include "utils/print.h"
#include "utils/misc.h"

BrainLog::BrainLog(Brain *argBrain) : brain(argBrain), log_tcp("robocup"), log_file("robocup")
{
    enable_log_tcp = brain->config->rerunLogEnableTCP;
    enable_log_file = brain->config->rerunLogEnableFile;
    if (enable_log_tcp)
    {
        rerun::Error err = log_tcp.connect(brain->config->rerunLogServerIP);
        if (err.is_err()) prtErr("Connect rerunLog server failed: " + err.description);
        // 注意这里，如果指定的地址里没有启动服务，err其实也不会报错，只会阻塞一定的时间（默认2s)，进不到这个分支
        // TODO：后续可以看看 relog 文档，把这里修一下，预期的是连不上服务要正常报错
    }
    

    if (enable_log_file)
    {
        brain->data->timeLastLogSave = brain->get_clock()->now();
        
        auto dir = brain->config->rerunLogLogDir;
        dir = gen_timestamped_filename(dir, format("_P%d_T%d", brain->config->playerId, brain->config->teamId));
        brain->config->rerunLogLogDir = dir;
        mkdir_if_not_exist(brain->config->rerunLogLogDir);
        
        auto file_name = gen_timestamped_filename(brain->config->rerunLogLogDir, ".rrd");
        auto saveError = log_file.save(file_name);
        if (saveError.is_err()) prtErr("Rerun log save Error: " + saveError.description);
    }

}

void BrainLog::setTimeNow()
{
    setTimeSeconds(brain->get_clock()->now().seconds());
}

void BrainLog::setTimeSeconds(double seconds)
{
    if (brain->config->rerunLogEnableTCP)
    {
        log_tcp.set_time_seconds("time", seconds);
    }

    if (brain->config->rerunLogEnableFile)
    {
        log_file.set_time_seconds("time", seconds);
    }

}

void BrainLog::logStatics()
{
    FieldDimensions &fd = brain->config->fieldDimensions;

    // draw lines
    vector<rerun::LineStrip2D> mapLines = {};
    for (int i = 0; i < brain->config->mapLines.size(); i++) {
        auto line = brain->config->mapLines[i].posToField;
        mapLines.push_back(rerun::LineStrip2D({
            {line.x0, -line.y0}, 
            {line.x1, -line.y1}, 
        }));
    }

    // draw center circle
    vector<rerun::Vec2D> circle = {{fd.circleRadius, 0}};
    for (int i = 0; i < 360; i++)
    {
        double r = fd.circleRadius;
        double theta = (i + 1) * M_PI / 180;
        circle.push_back(rerun::Vec2D{r * cos(theta), r * sin(theta)});
    }
    mapLines.push_back(rerun::LineStrip2D(circle));

    // draw penalty points
    mapLines.push_back(rerun::LineStrip2D({{fd.length / 2 - fd.penaltyDist - 0.2, 0}, {fd.length / 2 - fd.penaltyDist + 0.2, 0}}));
    mapLines.push_back(rerun::LineStrip2D({{fd.length / 2 - fd.penaltyDist, -0.2}, {fd.length / 2 - fd.penaltyDist, 0.2}}));
    mapLines.push_back(rerun::LineStrip2D({{-fd.length / 2 + fd.penaltyDist - 0.2, 0}, {-fd.length / 2 + fd.penaltyDist + 0.2, 0}}));
    mapLines.push_back(rerun::LineStrip2D({{-fd.length / 2 + fd.penaltyDist, -0.2}, {-fd.length / 2 + fd.penaltyDist, 0.2}}));

    // draw goals 
    mapLines.push_back(rerun::LineStrip2D({
        {fd.length / 2, fd.goalWidth / 2}, 
        {fd.length / 2 + 1.0, fd.goalWidth / 2}, 
        {fd.length / 2 + 1.0, -fd.goalWidth / 2}, 
        {fd.length / 2, -fd.goalWidth / 2}, 
    }));
    mapLines.push_back(rerun::LineStrip2D({
        {-fd.length / 2, fd.goalWidth / 2}, 
        {-fd.length / 2 - 1.0, fd.goalWidth / 2}, 
        {-fd.length / 2 - 1.0, -fd.goalWidth / 2}, 
        {-fd.length / 2, -fd.goalWidth / 2}, 
    }));

    
    log(
        "field/mapLines",
        rerun::LineStrips2D(mapLines)
            .with_colors({0xFFFFFFFF})
            .with_radii({0.03})
            .with_draw_order(0.0));

    
    vector<rerun::Vec2D> mapPoints;
    vector<string> mapPointLabels;
    for (int i = 0; i < brain->config->mapMarkings.size(); i++) {
        auto m = brain->config->mapMarkings[i];
        mapPoints.push_back(rerun::Vec2D{m.x, -m.y});
        mapPointLabels.push_back(m.name + " " + m.type);
    }
    // log(
    //     "field/mapPoints",
    //     rerun::Points2D(mapPoints)
    //        .with_labels(mapPointLabels)
    //        .with_colors({0xFFFFFFFF})
    //        .with_radii({0.05})
    // );


    // draw robot frame ref lines
    rerun::Collection<rerun::Vec2D> xaxis = {{-2, 0}, {2, 0}};
    rerun::Collection<rerun::Vec2D> yaxis = {{0, 2}, {0, -2}};
    rerun::Collection<rerun::Vec2D> border2m = {{-2, -2}, {-2, 2}, {2, 2}, {2, -2}, {-2, -2}};
    rerun::Collection<rerun::Vec2D> border1m = {{-1, -1}, {-1, 1}, {1, 1}, {1, -1}, {-1, -1}};
    rerun::Collection<rerun::Vec2D> angle = {{3 * cos(0.2), 3 * sin(0.2)}, {0, 0}, {3 * cos(0.2), 3 * sin(-0.2)}};
    vector<rerun::Collection<rerun::Vec2D>> refLines = {};
    refLines.push_back(xaxis);
    refLines.push_back(yaxis);
    refLines.push_back(border1m);
    refLines.push_back(border2m);
    refLines.push_back(angle);

    int nRef = 20; double spaceRef = 0.2; double lRef = 2.0;
    for (int i = 0; i <= nRef; i++) {
        double x = i * spaceRef;
        for (int j = 0; j <= nRef; j++) {
            double y = j * spaceRef;
            refLines.push_back({{x, -lRef}, {x, lRef}});
            refLines.push_back({{-x, -lRef}, {-x, lRef}});
            refLines.push_back({{lRef, -y}, {-lRef, -y}});
            refLines.push_back({{lRef, y}, {-lRef, y}});
        }
    }

    // log(
    //     "robotframe/lines",
    //     rerun::LineStrips2D(refLines)
    //         .with_colors({0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xAAAAAAFF})
    //         .with_radii({0.005, 0.005, 0.005, 0.002, 0.002, 0.002})
    //         .with_draw_order(0.0));
}

void BrainLog::prepare()
{
    setTimeSeconds(0);
    setTimeNow();
    logStatics();
}

void BrainLog::updateLogFilePath() {
    brain->data->timeLastLogSave = brain->get_clock()->now();
    auto file_name = gen_timestamped_filename(brain->config->rerunLogLogDir, ".rrd");
    auto saveError = log_file.save(file_name);
    if (saveError.is_err()) prtErr("Rerun log save Error: " + saveError.description);
    logStatics();
}

void BrainLog::logToScreen(string logPath, string text, u_int32_t color, double padding)
{
    log(
        logPath,
        rerun::Boxes2D::from_mins_and_sizes({{-padding, -padding}}, {{brain->config->camPixX + 2 * padding, brain->config->camPixY + 3 * padding}})
            .with_labels({text})
            .with_colors({color}));
}

rerun::LineStrip2D BrainLog::circle(float x, float y, float r, int nSegs) {
    vector<array<float, 2>> res = {};
    for (int i = 0; i < nSegs + 1; i++)
    {
        float theta = (i + 1) * 2 * M_PI / nSegs;
        res.push_back(array<float, 2>({x + r * cos(theta), y + r * sin(theta)}));
    }
    return rerun::LineStrip2D(res);
}

rerun::LineStrip2D BrainLog::crosshair(float x, float y, float r) {
    return rerun::LineStrip2D({{x, y - r}, {x, y + r }, {x - r, y}, {x + r, y}});
}

void BrainLog::logRobot(string logPath, Pose2D pose, u_int32_t color, string label, bool draw2mCircle) {
    vector<rerun::LineStrip2D> lines = {};
    auto dirLine = rerun::LineStrip2D({{pose.x, -pose.y}, {pose.x + 0.6 * cos(-pose.theta), -pose.y + 0.6 * sin(-pose.theta)}});
    auto body = circle(pose.x, -pose.y, 0.3);
    vector<double> widths = {};
    lines.push_back(body); widths.push_back(0.01);
    lines.push_back(dirLine); widths.push_back(0.01);
    if (draw2mCircle) {
        lines.push_back(circle(pose.x, -pose.y, 2.0));
        widths.push_back(0.01);
    }


    
    log(logPath, 
        rerun::LineStrips2D(rerun::Collection<rerun::components::LineStrip2D>(lines))
        .with_radii(widths)
        .with_colors(color)
        .with_labels({label})
    );
}

void BrainLog::logBall(string logPath, Point pos, u_int32_t color, bool detected, bool known) {
    vector<rerun::LineStrip2D> lines = {};
    lines.push_back(circle(pos.x, -pos.y, 0.1)); // ball
    lines.push_back(rerun::LineStrip2D({{pos.x, -pos.y - 0.5}, {pos.x, -pos.y + 0.5}})); // corsshair
    lines.push_back(rerun::LineStrip2D({{pos.x - 0.5, -pos.y}, {pos.x + 0.5, -pos.y}})); // crosshair
    if (known) lines.push_back(circle(pos.x, -pos.y, 0.2));
    if (detected) lines.push_back(circle(pos.x, -pos.y, 0.3));
    
    log(logPath,
        rerun::LineStrips2D(rerun::Collection<rerun::components::LineStrip2D>(lines))
       .with_radii({0.01, 0.01, 0.01})
       .with_colors(color));
}
