# following include/RoboCupGameControllData.h's RoboCupGameControlData structure

char[4] header              # header to identify the structure
uint16 version              # version of the data structure
uint8 packet_number          # number incremented with each packet sent (with wraparound)
uint8 players_per_team        # the number of players on a team
uint8 game_type              # type of the game (GAME_ROUNDROBIN, GAME_PLAYOFF, GAME_DROPIN)
uint8 state                 # state of the game (STATE_READY, STATE_PLAYING, etc)
uint8 first_half             # 1 = game in first half, 0 otherwise
uint8 kick_off_team           # the team number of the next team to kick off or DROPBALL
uint8 secondary_state        # extra state information - (STATE2_NORMAL, STATE2_PENALTYSHOOT, etc)
char[4] secondary_state_info  # Extra info on the secondary state
uint8 drop_in_team            # number of team that caused last drop in
uint16 drop_in_time           # number of seconds passed since the last drop in. -1 (0xffff) before first dropin
uint16 secs_remaining        # estimate of number of seconds remaining in the half
uint16 secondary_time        # number of seconds shown as secondary time (remaining ready, until free ball, etc)
TeamInfo[2] teams