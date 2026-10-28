# 跟 include/RoboCupGameControllData.h 中的 TeamInfo 保持一致的结构
# 稍注意一下，coach_message 和 players 在这里定义为不定长数组，更符合语义

uint8 team_number           # unique team number
uint8 field_player_colour   # colour of the team
uint8 goalkeeper_colour     # GameController v20 goalkeeper jersey colour
uint8 goalkeeper            # GameController v20 goalkeeper player number
uint8 score                 # team's score
uint8 penalty_shot          # penalty shot counter
uint16 single_shots         # bits represent penalty shot success
uint16 message_budget       # GameController v20 remaining team-message budget
uint8 coach_sequence        # sequence number of the coach's message
uint8[] coach_message       # the coach's message to the team (length=253)
RobotInfo coach
RobotInfo[] players         # legacy packets carry 11 slots; v20 packets carry 20
