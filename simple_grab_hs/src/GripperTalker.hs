module GripperTalker (main) where

import Ros.Node (Topic)
import Ros.Node (topicRate)
import Ros.Node (runNode)
import Ros.Node (advertise)
import Ros.Topic (repeatM)
import Ros.Pr2_controllers_msgs.Pr2GripperCommandGoal
import Ros.Pr2_controllers_msgs.Pr2GripperCommandActionGoal
import Ros.Pr2_controllers_msgs.Pr2GripperCommand
import qualified Ros.Std_msgs.Header as H
import qualified Ros.Actionlib_msgs.GoalID as GID
import Data.Default.Generics (def)

open :: Double
open = 0.09

closed :: Double
closed = 0.0

moveGripper :: Topic IO Pr2GripperCommandActionGoal
moveGripper = repeatM msg
  where msg = return $ (def :: Pr2GripperCommandActionGoal){
          {-
          header= H.Header {H.seq= 0,
                            H.stamp = (0,0),
                            H.frame_id= ""},
          goal_id= GID.GoalID {
            GID.stamp = (0,0),
            GID.id = ""
            }
          ,
-}
          goal=Pr2GripperCommandGoal{
             command = Pr2GripperCommand{
                position = closed, max_effort = (-1)
                }
             }

          }

main :: IO ()
main = runNode "GripperTakler" $ advertise topicName (topicRate 100 moveGripper)
  where topicName = "/r_gripper_controller/gripper_action/goal"
--  where topicName = "gripper_goal"
