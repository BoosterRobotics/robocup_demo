# following include/RoboCupGameControllData.h's RoboCupGameControlData structure

uint8 team_number           # unique team number
uint8 field_player_colour   # colour of the team
uint8 score                 # team's score
uint8 penalty_shot          # penalty shot counter
uint16 single_shots         # bits represent penalty shot success
uint8 coach_sequence        # sequence number of the coach's message
uint8[] coach_message       # the coach's message to the team (length=253)
RobotInfo coach
RobotInfo[] players         # the team's players (length=11)