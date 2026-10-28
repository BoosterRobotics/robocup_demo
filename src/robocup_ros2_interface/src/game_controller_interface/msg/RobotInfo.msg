# 跟 include/RoboCupGameControllData.h 中的 RobotInfo 保持一致的结构

uint8 penalty                   # penalty state of the player
uint8 secs_till_unpenalised     # estimate of time till unpenalised
uint8 number_of_warnings        # number of warnings
uint8 yellow_card_count         # number of yellow cards
uint8 red_card_count            # number of red cards
uint8 cautions                  # GameController v20 caution count
bool goal_keeper                # flags if robot is goal keeper
