module GripWhenBroken (main) where

import Ros.Node (subscribe)
import Ros.Node (runNode)
import qualified Ros.Pr2_pretouch_sensor_optical.OpticalBeams as OB
import qualified Data.Vector.Storable as Vec

import Ros.Pr2_controllers_msgs.Pr2GripperCommandGoal
import Ros.Pr2_controllers_msgs.Pr2GripperCommandActionGoal
import Ros.Pr2_controllers_msgs.Pr2GripperCommand
import qualified Ros.Std_msgs.Header as H
import qualified Ros.Actionlib_msgs.GoalID as GID
import Ros.Node (advertise)
import Data.Vector.Storable ((!))

-- Types
data OpenOrClosed = Open | Closed
type Position = Double

--Newtons of gripper force, 30 is a minimum
data Effort = FullEffort | LimitedEffort Double

--Constants
rightTopicName :: String
rightTopicName = "/optical/right"

leftTopicName :: String
leftTopicName = "/optical/left"

openPosition :: Position
openPosition = 0.09

closedPosition :: Position
closedPosition = 0.03
-- Functions

showMsg :: OB.OpticalBeams -> IO ()
showMsg  = putStrLn . ("I heard ya " ++ ) . show

--Returns if the gripper should be open or closed              
sensorMsgToOpenOrClosed:: OB.OpticalBeams -> OpenOrClosed
sensorMsgToOpenOrClosed =

--  brokenToOpenClosed . Vec.elem True . Vec.map word8ToBool . OB.broken
  brokenToOpenClosed . word8ToBool . (!1) . (OB.broken)
  where
    brokenToOpenClosed True = Closed
    brokenToOpenClosed False = Open
    word8ToBool 0 = False
    word8ToBool _ = True
  
makeGripperCommandActionGoal :: Position -> Effort -> Pr2GripperCommandActionGoal
makeGripperCommandActionGoal pos effort =
  Pr2GripperCommandActionGoal{  
    header= H.Header {H.seq= 0,
                      H.stamp = (0,0),
                      H.frame_id= ""},
    goal_id= GID.GoalID {
      GID.stamp = (0,0),
      GID.id = ""
      },
    goal=Pr2GripperCommandGoal{
      command = Pr2GripperCommand{
         position = pos,
         max_effort = case effort of
           FullEffort -> -1
           LimitedEffort x -> x
         }
      }
    }

--If the bool is true, then open sensors
makeGoalFromBool :: OpenOrClosed -> Pr2GripperCommandActionGoal
makeGoalFromBool Closed = makeGripperCommandActionGoal closedPosition FullEffort
makeGoalFromBool Open = makeGripperCommandActionGoal openPosition FullEffort

main :: IO ()
main = runNode "SensorSubscriber" $
       do sensorMessages <- subscribe leftTopicName
          let processedMessages = fmap (makeGoalFromBool . sensorMsgToOpenOrClosed) sensorMessages
          advertise leftGripperTopicName processedMessages
          where
            rightGripperTopicName = "/r_gripper_controller/gripper_action/goal"
            leftGripperTopicName = "/l_gripper_controller/gripper_action/goal"
              
       
