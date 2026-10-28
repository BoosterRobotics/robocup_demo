# 跟 include/RoboCupGameControllData.h 中的 RoboCupGameControlData 保持一致的结构

char[4] header              # header to identify the structure
uint16 version              # version of the data structure
uint8 packet_number          # number incremented with each packet sent (with wraparound)
uint8 players_per_team        # the number of players on a team
uint8 game_type              # type of the game (GAME_ROUNDROBIN, GAME_PLAYOFF, GAME_DROPIN)
bool stopped                 # GameController v20: play is temporarily stopped
uint8 game_phase             # GameController v20: normal, penalty shoot-out, extra time, timeout
uint8 set_play               # GameController v20: active set play
uint8 state                 # state of the game (STATE_READY, STATE_PLAYING, etc)
uint8 first_half             # 1 = game in first half, 0 otherwise
uint8 kick_off_team           # the team number of the next team to kick off or DROPBALL
uint8 secondary_state        # extra state information - (STATE2_NORMAL, STATE2_PENALTYSHOOT, etc)
char[4] secondary_state_info  # Extra info on the secondary state
uint8 drop_in_team            # number of team that caused last drop in
uint16 drop_in_time           # number of seconds passed since the last drop in. -1 (0xffff) before first dropin
int16 secs_remaining         # estimate of number of seconds remaining in the half; v20 permits negatives
int16 secondary_time        # number of seconds shown as secondary time; v20 permits negatives
TeamInfo[2] teams
