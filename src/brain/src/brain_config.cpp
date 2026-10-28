#include "brain_config.h"
#include "utils/print.h"
#include <cctype>

void BrainConfig::calcMapLines() {
    auto fd = fieldDimensions;
    mapLines.clear();
    
    FieldLine oppoGoalLine;
    oppoGoalLine.posToField = {fd.length / 2, -fd.width / 2, fd.length / 2, fd.width / 2};
    oppoGoalLine.half = LineHalf::Opponent;
    oppoGoalLine.side = LineSide::NA;
    oppoGoalLine.dir = LineDir::Horizontal;
    oppoGoalLine.type = LineType::GoalLine;
    mapLines.push_back(oppoGoalLine);

    FieldLine selfGoalLine;
    selfGoalLine.posToField = {-fd.length / 2, -fd.width / 2, -fd.length / 2, fd.width / 2};
    selfGoalLine.half = LineHalf::Self;
    selfGoalLine.side = LineSide::NA;
    selfGoalLine.dir = LineDir::Horizontal;
    selfGoalLine.type = LineType::GoalLine;
    mapLines.push_back(selfGoalLine);

    FieldLine leftTouchLine;
    leftTouchLine.posToField = {-fd.length / 2, fd.width / 2, fd.length / 2, fd.width / 2};
    leftTouchLine.half = LineHalf::NA;
    leftTouchLine.side = LineSide::Left;
    leftTouchLine.dir = LineDir::Vertical;
    leftTouchLine.type = LineType::TouchLine;
    mapLines.push_back(leftTouchLine);

    FieldLine rightTouchLine;
    rightTouchLine.posToField = {-fd.length / 2, -fd.width / 2, fd.length / 2, -fd.width / 2};
    rightTouchLine.half = LineHalf::NA;
    rightTouchLine.side = LineSide::Right;
    rightTouchLine.dir = LineDir::Vertical;
    rightTouchLine.type = LineType::TouchLine;
    mapLines.push_back(rightTouchLine);

    FieldLine middleLine;
    middleLine.posToField = {0, -fd.width / 2, 0, fd.width / 2};
    middleLine.half = LineHalf::NA;
    middleLine.side = LineSide::NA;
    middleLine.dir = LineDir::Horizontal;
    middleLine.type = LineType::MiddleLine;
    mapLines.push_back(middleLine);

    FieldLine goalAreaLO;
    goalAreaLO.posToField = {fd.length / 2, -fd.goalAreaWidth / 2, fd.length / 2 - fd.goalAreaLength, -fd.goalAreaWidth / 2};
    goalAreaLO.half = LineHalf::Opponent;
    goalAreaLO.side = LineSide::Left;
    goalAreaLO.dir = LineDir::Vertical;
    goalAreaLO.type = LineType::GoalArea;
    mapLines.push_back(goalAreaLO);

    FieldLine goalAreaRO;
    goalAreaRO.posToField = {fd.length / 2, fd.goalAreaWidth / 2, fd.length / 2 - fd.goalAreaLength, fd.goalAreaWidth / 2};
    goalAreaRO.half = LineHalf::Opponent;
    goalAreaRO.side = LineSide::Right;
    goalAreaRO.dir = LineDir::Vertical;
    goalAreaRO.type = LineType::GoalArea;
    mapLines.push_back(goalAreaRO);

    FieldLine goalAreaHO;
    goalAreaHO.posToField = {fd.length / 2 - fd.goalAreaLength, -fd.goalAreaWidth / 2, fd.length / 2 - fd.goalAreaLength, fd.goalAreaWidth / 2};
    goalAreaHO.half = LineHalf::Opponent;
    goalAreaHO.side = LineSide::NA;
    goalAreaHO.dir = LineDir::Horizontal;
    goalAreaHO.type = LineType::GoalArea;
    mapLines.push_back(goalAreaHO);

    FieldLine penaltyAreaLO;
    penaltyAreaLO.posToField = {fd.length / 2, -fd.penaltyAreaWidth / 2, fd.length / 2 - fd.penaltyAreaLength, -fd.penaltyAreaWidth / 2};
    penaltyAreaLO.half = LineHalf::Opponent;
    penaltyAreaLO.side = LineSide::Left;
    penaltyAreaLO.dir = LineDir::Vertical;
    penaltyAreaLO.type = LineType::PenaltyArea;
    mapLines.push_back(penaltyAreaLO);

    FieldLine penaltyAreaRO;
    penaltyAreaRO.posToField = {fd.length / 2, fd.penaltyAreaWidth / 2, fd.length / 2 - fd.penaltyAreaLength, fd.penaltyAreaWidth / 2};
    penaltyAreaRO.half = LineHalf::Opponent;
    penaltyAreaRO.side = LineSide::Right;
    penaltyAreaRO.dir = LineDir::Vertical;
    penaltyAreaRO.type = LineType::PenaltyArea;
    mapLines.push_back(penaltyAreaRO);

    FieldLine penaltyAreaHO;
    penaltyAreaHO.posToField = {fd.length / 2 - fd.penaltyAreaLength, -fd.penaltyAreaWidth / 2, fd.length / 2 - fd.penaltyAreaLength, fd.penaltyAreaWidth / 2};
    penaltyAreaHO.half = LineHalf::Opponent;
    penaltyAreaHO.side = LineSide::NA;
    penaltyAreaHO.dir = LineDir::Horizontal;
    penaltyAreaHO.type = LineType::PenaltyArea;
    mapLines.push_back(penaltyAreaHO);

    FieldLine goalAreaLS;
    goalAreaLS.posToField = {-fd.length / 2, -fd.goalAreaWidth / 2, -fd.length / 2 + fd.goalAreaLength, -fd.goalAreaWidth / 2};
    goalAreaLS.half = LineHalf::Self;
    goalAreaLS.side = LineSide::Left;
    goalAreaLS.dir = LineDir::Vertical;
    goalAreaLS.type = LineType::GoalArea;
    mapLines.push_back(goalAreaLS);

    FieldLine goalAreaRS;
    goalAreaRS.posToField = {-fd.length / 2, fd.goalAreaWidth / 2, -fd.length / 2 + fd.goalAreaLength, fd.goalAreaWidth / 2};
    goalAreaRS.half = LineHalf::Self;
    goalAreaRS.side = LineSide::Right;
    goalAreaRS.dir = LineDir::Vertical;
    goalAreaRS.type = LineType::GoalArea;
    mapLines.push_back(goalAreaRS);

    FieldLine goalAreaHS;
    goalAreaHS.posToField = {-fd.length / 2 + fd.goalAreaLength, -fd.goalAreaWidth / 2, -fd.length / 2 + fd.goalAreaLength, fd.goalAreaWidth / 2};
    goalAreaHS.half = LineHalf::Self;
    goalAreaHS.side = LineSide::NA;
    goalAreaHS.dir = LineDir::Horizontal;
    goalAreaHS.type = LineType::GoalArea;
    mapLines.push_back(goalAreaHS);

    FieldLine penaltyAreaLS;
    penaltyAreaLS.posToField = {-fd.length / 2, -fd.penaltyAreaWidth / 2, -fd.length / 2 + fd.penaltyAreaLength, -fd.penaltyAreaWidth / 2};
    penaltyAreaLS.half = LineHalf::Self;
    penaltyAreaLS.side = LineSide::Left;
    penaltyAreaLS.dir = LineDir::Vertical;
    penaltyAreaLS.type = LineType::PenaltyArea;
    mapLines.push_back(penaltyAreaLS);

    FieldLine penaltyAreaRS;
    penaltyAreaRS.posToField = {-fd.length / 2, fd.penaltyAreaWidth / 2, -fd.length / 2 + fd.penaltyAreaLength, fd.penaltyAreaWidth / 2};
    penaltyAreaRS.half = LineHalf::Self;
    penaltyAreaRS.side = LineSide::Right;
    penaltyAreaRS.dir = LineDir::Vertical;
    penaltyAreaRS.type = LineType::PenaltyArea;
    mapLines.push_back(penaltyAreaRS);

    FieldLine penaltyAreaHS;
    penaltyAreaHS.posToField = {-fd.length / 2 + fd.penaltyAreaLength, -fd.penaltyAreaWidth / 2, -fd.length / 2 + fd.penaltyAreaLength, fd.penaltyAreaWidth / 2};
    penaltyAreaHS.half = LineHalf::Self;
    penaltyAreaHS.side = LineSide::NA;
    penaltyAreaHS.dir = LineDir::Horizontal;
    penaltyAreaHS.type = LineType::PenaltyArea;
    mapLines.push_back(penaltyAreaHS);
}

void BrainConfig::calcMapMarkings() {
    auto fd = fieldDimensions;
    mapMarkings.clear();

    vector<string> halves = {"S", "O"};
    vector<string> sides = {"L", "R"};
    for (auto half : halves) {
        double xSign = half == "S" ? -1.0 : 1.0;
        for (auto side : sides) {
            double ySign = side == "L"? 1.0 : -1.0;
            
            mapMarkings.push_back(MapMarking({
                xSign * fd.length / 2,
                ySign * fd.width / 2,
                "LCross",
                "L" + half + side + "B"
            }));

            mapMarkings.push_back(MapMarking({
                xSign * fd.length / 2,
                ySign * fd.penaltyAreaWidth / 2,
                "TCross",
                "T" + half + side + "P"
            }));

            mapMarkings.push_back(MapMarking({
                xSign * fd.length / 2,
                ySign * fd.goalAreaWidth / 2,
                "TCross",
                "T" + half + side + "G"
            }));

            mapMarkings.push_back(MapMarking({
                xSign * (fd.length / 2 - fd.penaltyAreaLength),
                ySign * fd.penaltyAreaWidth / 2,
                "LCross",
                "L" + half + side + "P"
            }));

            mapMarkings.push_back(MapMarking({
                xSign * (fd.length / 2 - fd.goalAreaLength),
                ySign * fd.goalAreaWidth / 2,
                "LCross",
                "L" + half + side + "G"
            }));
        }
    }

    // 中线上四点
    for (auto side: sides) {
        double ySign = side == "L"? 1.0 : -1.0;

        mapMarkings.push_back(MapMarking({
            0,
            ySign * fd.width / 2,
            "TCross",
            "TM" + side + "B"
        }));

        mapMarkings.push_back(MapMarking({
            0,
            ySign * fd.circleRadius,
            "XCross",
            "XM" + side + "C"
        }));
    }

    // 两个 penalty 点
    for (auto half : halves) {
        double xSign = half == "S"? -1.0 : 1.0;

        mapMarkings.push_back(MapMarking({
            xSign * (fd.length / 2 - fd.penaltyDist),
            0,
            "PenaltyPoint",
            "P" + half + "MP"
        }));
    }
}

void BrainConfig::handle()
{
    // playerStartPos[left, right]
    if (playerStartPos != "left" && playerStartPos != "right")
    {
        throw invalid_argument("palyer_start_pos must be one of [left, right]. Got: " + playerStartPos);
    }

    // playerRole [striker, goal_keeper]
    if (playerRole != "striker" && playerRole != "goal_keeper")
    {
        throw invalid_argument("player_role must be one of [striker, goal_keeper]. Got: " + playerRole);
    }

    // playerId
    if (playerId < 1 || playerId > HL_MAX_NUM_PLAYERS)
    {
        throw invalid_argument("[Error] player_id must be one of [1, .. 11]. Got: " + to_string(playerId));
    }

    // fieldType [adult_size, kid_size]
    if (fieldType == "adult_size")
    {
        fieldDimensions = FD_ADULTSIZE;
    }
    else if (fieldType == "kid_size")
    {
        fieldDimensions = FD_KIDSIZE;
    }
        else if (fieldType == "robo_league")
    {
        fieldDimensions = FD_ROBOLEAGUE;
    }
    else
    {
        throw invalid_argument("[Error] fieldType must be one of [adult_size, kid_size, robo_league]. Got: " + fieldType);
    }

    string getUpVersionLower;
    getUpVersionLower.reserve(recoveryGetUpVersion.size());
    for (unsigned char c : recoveryGetUpVersion) {
        getUpVersionLower.push_back(static_cast<char>(std::tolower(c)));
    }
    if (getUpVersionLower != "kv1" && getUpVersionLower != "v1" &&
        getUpVersionLower != "kv2" && getUpVersionLower != "v2") {
        throw invalid_argument(
            "[Error] recovery.get_up_version must be one of [kV1, kV2]. Got: " +
            recoveryGetUpVersion);
    }

    string verLower;
    verLower.reserve(RLVisionKickVisualKickVersion.size());
    for (unsigned char c : RLVisionKickVisualKickVersion) {
        verLower.push_back(static_cast<char>(std::tolower(c)));
    }
    if (verLower != "kv1" && verLower != "v1" && verLower != "kv2" && verLower != "v2") {
        throw invalid_argument("[Error] RLVisionKick.visual_kick_version must be one of [kV1, kV2]. Got: " + RLVisionKickVisualKickVersion);
    }

    calcMapLines();
    calcMapMarkings();
}

void BrainConfig::print(ostream &os)
{
    os << "Configs:" << endl;
    os << "----------------------------------------" << endl;
    os << "Game:" << endl;
    os << "    teamId = " << teamId << endl;
    os << "    playerId = " << playerId << endl;
    os << "    fieldType = " << fieldType << endl;
    os << "    playerRole = " << playerRole << endl;
    os << "    playerStartPos = " << playerStartPos << endl;
    os << "    treatPersonAsRobot = " << treatPersonAsRobot << endl;
    os << "    numOfPlayers = " << numOfPlayers << endl;
    os << "----------------------------------------" << endl;
    os << "Robot:" << endl;
    os << "    robotHeight = " << robotHeight << endl;
    os << "    robotOdomFactor = " << robotOdomFactor << endl;
    os << "    vxFactor = " << vxFactor << endl;
    os << "    yawOffset = " << yawOffset << endl;
    os << "    vxLimit = " << vxLimit << endl;
    os << "    vyLimit = " << vyLimit << endl;
    os << "    vthetaLimit = " << vthetaLimit << endl;
    os << "----------------------------------------" << endl;
    os << "Strategy:" << endl;
    os << "    ballConfidenceThreshold = " << ballConfidenceThreshold << endl;
    os << "    ballConfidenceDecayRate = " << ballConfidenceDecayRate << endl;

    os << "----------------------------------------" << endl;
    os << "Locator:" << endl;
    os << "    pfMinMarkerCnt = " << pfMinMarkerCnt << endl;
    os << "    pfMaxResidual = " << pfMaxResidual << endl;
    os << "----------------------------------------" << endl;
    os << "Communication:" << endl;
    os << "    enableCom = " << enableCom << endl;
    os << "----------------------------------------" << endl;
    os << "RerunLog:" << endl;
    os << "    enableTCP = " << rerunLogEnableTCP << endl;
    os << "    serverIP = " << rerunLogServerIP << endl;
    os << "    enableFile = " << rerunLogEnableFile << endl;
    os << "    logDir = " << rerunLogLogDir << endl;
    os << "    maxFileMinutes = " << rerunLogMaxFileMins << endl;
    os << "    imgInterval = " << rerunLogImgInterval << endl;
    os << "----------------------------------------" << endl;
    os << "Sound:" << endl;
    os << "    enable = " << soundEnable << endl;
    os << "    soundPack = " << soundPack << endl;
    os << "----------------------------------------" << endl;
    os << "Recovery:" << endl;
    os << "    getUpVersion = " << recoveryGetUpVersion << endl;
    os << "----------------------------------------" << endl;
    os << "RLVisionKick:" << endl;
    os << "    visualKickVersion = " << RLVisionKickVisualKickVersion << endl;
    os << "----------------------------------------" << endl;
}
