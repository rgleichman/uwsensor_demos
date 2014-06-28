module AngleGripper
       (main
       )
       where
import Prelude hiding (filter, or)
import Ros.Node (Topic)
import qualified Ros.Geometry_msgs.Pose as POS
import qualified Ros.Geometry_msgs.Point as POI
import qualified Ros.Geometry_msgs.Quaternion as QUA
import Ros.Node (runNode)
import Ros.Node (subscribe)
import qualified MoveToPose as MTP
import Ros.Node (topicRate)
import Ros.Node (advertise)
import Ros.Arm_navigation_msgs.MoveArmActionGoal (MoveArmActionGoal)
import qualified Ros.Pr2_pretouch_sensor_optical.OpticalBeams as OB
import qualified Data.Vector.Storable as VEC


armOut :: POS.Pose
armOut = POS.Pose{
  POS.position = POI.Point{
     POI.x = 0.75
     ,POI.y = -0.188
     ,POI.z = 0
     }
  ,POS.orientation = vertical
  }

armIn :: POS.Pose
armIn = POS.Pose{
  POS.position = POI.Point{
     POI.x = 0.5
     ,POI.y = -0.188
     ,POI.z = 0
     }
  ,POS.orientation = vertical
  }

vertical :: QUA.Quaternion
vertical = QUA.Quaternion{
  QUA.x = 1.0
  ,QUA.y = 0.0
  ,QUA.z = 0.0
  ,QUA.w = 1.0
  }
         

main :: IO ()
main = runNode "AngleGripper" $ do
  statusMsgs <- subscribe leftStatusTopicName
  sensorMessages <- subscribe leftSensorTopicName
  let goalMsgs= fmap makeGoal sensorMessages
      filteredGoalMsgs = MTP.filterNoActive statusMsgs goalMsgs
  advertise leftMoveArmGoalTopicName (topicRate 10 filteredGoalMsgs)
    where rightMoveArmGoalTopicName = "/move_right_arm/goal"
          rightStatusTopicName = "/move_right_arm/status"
          rightSensorTopicName = "/optical/right"
          leftSensorTopicName = "/optical/left"
          leftMoveArmGoalTopicName = "/move_left_arm/goal"
          leftStatusTopicName = "/move_left_arm/status"
          

makeGoal :: OB.OpticalBeams -> MoveArmActionGoal
makeGoal beams = MTP.makeMoveArmActionGoal $ if anyBroken beams then (lArmIn, MTP.LeftArm) else (lArmOut, MTP.LeftArm)
  where anyBroken = VEC.or . (OB.broken)

lArmOut :: POS.Pose
lArmOut = POS.Pose{
  POS.position = POI.Point{
     POI.x = 0.75
     ,POI.y = 0.188
     ,POI.z = 0
     }
  ,POS.orientation = vertical
  }

lArmIn :: POS.Pose
lArmIn = POS.Pose{
  POS.position = POI.Point{
     POI.x = 0.70
     ,POI.y = 0.188
     ,POI.z = 0
     }
  ,POS.orientation = vertical
  }
