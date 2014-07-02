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
import Data.Monoid
import Data.Monoid
import qualified Ros.Topic as TOP
import Ros.Node (Node)
import Control.Applicative ((<*>))
import Control.Applicative ((<$>))
import qualified Ros.Actionlib_msgs.GoalStatusArray as GOS

armOut :: POS.Pose
armOut = POS.Pose{
  POS.position = POI.Point{
     POI.x = 0.75
     ,POI.y = -0.188
     ,POI.z = 0
     }
  ,POS.orientation = MTP.vertical
  }

armIn :: POS.Pose
armIn = POS.Pose{
  POS.position = POI.Point{
     POI.x = 0.5
     ,POI.y = -0.188
     ,POI.z = 0
     }
  ,POS.orientation = MTP.vertical
  }

lArmOut :: POS.Pose
lArmOut = POS.Pose{
  POS.position = POI.Point{
     POI.x = 0.75
     ,POI.y = 0.188
     ,POI.z = 0
     }
  ,POS.orientation = MTP.vertical
  }

lArmIn :: POS.Pose
lArmIn = POS.Pose{
  POS.position = lArmInPoint
  ,POS.orientation = MTP.vertical
  }

lArmInPoint = POI.Point{
  POI.x = 0.70
  ,POI.y = 0.188
  ,POI.z = 0
  }

newtype MPoint = MPoint {unMPoint :: POI.Point}

instance Monoid MPoint where
  mempty = MPoint POI.Point{
    POI.x = 0
    ,POI.y = 0
    ,POI.z = 0
    }
  mappend (MPoint (POI.Point x1 y1 z1)) (MPoint (POI.Point x2 y2 z2)) = MPoint $ POI.Point (x1+x2) (y1+y2) (z1+z2)

data Direction = DirForward | DirBackward

integrate :: (Monad m, Monoid a) => a -> Topic m a -> Topic m a
integrate initialVal  = TOP.scan mappend initialVal


main :: IO ()
main = runNode "AngleGripper" $ goToTwoPose
          

goToTwoPose :: Node ()
goToTwoPose = do
  statusMsgs <- subscribe leftStatusTopicName
  sensorMessages <- subscribe leftSensorTopicName
  let goalMsgs= fmap makeGoal sensorMessages
      filteredGoalMsgs = MTP.filterNoActive statusMsgs goalMsgs
  advertise leftMoveArmGoalTopicName (topicRate 10 filteredGoalMsgs)

{-
* get status
* get sensor messages
* convert status messages to acceptingGoals indicator
* limit the sensor messages to acceptingGoals (gate sensors acceptingGoals)
* convert sensor messages to directions (ie. forwards, back, leftTurn, rightTurn)
* convert directions to POI.Point values
* integrate the POI.Point values to get the actual position
* map over the POI.Point values to make MoveArmActionGoal goal messages
-}
forwardBackControl :: Node ()
forwardBackControl = do
  status <- subscribe leftStatusTopicName
  sensor <- subscribe leftSensorTopicName
  let limitedSensor = MTP.filterNoActive status sensor
      pointVectors = fmap (directionsToPoints . sensorToDirectinos) limitedSensor
      position = fmap unMPoint $ integrate (MPoint lArmInPoint) (fmap MPoint pointVectors)
      goals = fmap (positionsToGoals MTP.vertical) position
  advertise leftMoveArmGoalTopicName (topicRate 10 goals)
      

positionsToGoals :: QUA.Quaternion -> POI.Point -> MoveArmActionGoal
positionsToGoals = undefined

directionsToPoints = undefined

sensorToDirectinos :: GOS.GoalStatusArray -> Direction
sensorToDirectinos = undefined

rightMoveArmGoalTopicName = "/move_right_arm/goal"
rightStatusTopicName = "/move_right_arm/status"
rightSensorTopicName = "/optical/right"
leftSensorTopicName = "/optical/left"
leftMoveArmGoalTopicName = "/move_left_arm/goal"
leftStatusTopicName = "/move_left_arm/status"

makeGoal :: OB.OpticalBeams -> MoveArmActionGoal
makeGoal beams = MTP.makeMoveArmActionGoal $ if anyBroken beams then (lArmIn, MTP.LeftArm) else (lArmOut, MTP.LeftArm)
  where anyBroken = VEC.or . (OB.broken)

