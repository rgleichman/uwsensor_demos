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
import qualified Ros.Topic as TOP
import Ros.Node (Node)
import qualified Ros.Std_msgs.Int64 as I
import qualified Ros.Topic.Util as TU
import qualified Ros.Actionlib_msgs.GoalID as GID
import Data.Default.Generics (def)

-- CONSTANTS
rightMoveArmGoalTopicName :: String
rightMoveArmGoalTopicName = "/move_right_arm/goal"
rightStatusTopicName :: String
rightStatusTopicName = "/move_right_arm/status"
rightSensorTopicName :: String
rightSensorTopicName = "/optical/right"
leftSensorTopicName :: String
leftSensorTopicName = "/optical/left"
leftMoveArmGoalTopicName :: String
leftMoveArmGoalTopicName = "/move_left_arm/goal"
leftStatusTopicName :: String
leftStatusTopicName = "/move_left_arm/status"
leftArmCancelName :: String
leftArmCancelName = "/move_left_arm/cancel"

-- x+ = forwards away from trunk parallel to ground
-- y+ = leftwards, 0 is midline of robot
-- z+ = up
--a = Ros
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

lArmInPoint :: POI.Point
lArmInPoint = POI.Point{
  POI.x = 0.5
  ,POI.y = 0.188
  ,POI.z = 0
  }

-- TYPES
newtype MPoint = MPoint {unMPoint :: POI.Point}

instance Monoid MPoint where
  mempty = MPoint POI.Point{
    POI.x = 0
    ,POI.y = 0
    ,POI.z = 0
    }
  mappend (MPoint (POI.Point x1 y1 z1)) (MPoint (POI.Point x2 y2 z2)) = MPoint $ POI.Point (x1+x2) (y1+y2) (z1+z2)

data Direction = DirForward | DirBackward

-- MAINS

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
* clamp the position so it is inside the workspace of the robot
* map over the POI.Point values to make MoveArmActionGoal goal messages
-}
forwardBackControl :: Node ()
forwardBackControl = do
  status <- subscribe leftStatusTopicName
  sensor <- subscribe leftSensorTopicName
  let limitedSensor = MTP.filterNoActive status sensor
      pointVectors = fmap (directionsToPoints . sensorToDirectinos) limitedSensor
      position = fmap unMPoint $ integrateWithBounds
                 (MPoint . clampPosition . unMPoint)
                 (MPoint lArmInPoint)
                 (fmap MPoint pointVectors)
      goals = fmap (positionsToGoals MTP.LeftArm MTP.vertical) position
  advertise leftMoveArmGoalTopicName (topicRate 10 goals)

{-
* get status
* get sensor messages
* convert status messages to acceptingGoals indicator
* limit the sensor messages to acceptingGoals (gate sensors acceptingGoals)
* convert sensor messages to directions (ie. forwards, back, leftTurn, rightTurn)
* convert directions to POI.Point values
* integrate the POI.Point values to get the actual position
* clamp the position so it is inside the workspace of the robot
* map over the POI.Point values to make MoveArmActionGoal goal messages
* create an event topic 'change' that occurs when the sensorBrokeness changes values
* publish MoveArmAction stop messages, but gate it to change so it stops the arm whenever the senor switches between broken and not broken
-}
stopEarly :: Node ()
stopEarly = do
  status <- subscribe leftStatusTopicName
  sensor <- subscribe leftSensorTopicName
  let 
      broken = fmap anyBroken sensor
      limitedBroken = MTP.filterNoActive status broken
      pointVectors = fmap (directionsToPoints . brokenToDirection) limitedBroken
      position = fmap unMPoint $ integrateWithBounds
                 (MPoint . clampPosition . unMPoint)
                 (MPoint lArmInPoint)
                 (fmap MPoint pointVectors)
      goals = fmap (positionsToGoals MTP.LeftArm MTP.vertical) position
      --caneling
      change = TOP.filter (uncurry  (/=)) $ TU.consecutive broken
      changeOnlyWhenActive = MTP.filterActive status change
      cancels = fmap makeCancel changeOnlyWhenActive
      
  advertise leftMoveArmGoalTopicName (topicRate 10 goals)
--  advertise rightSensorTopicName (topicRate 10 goals)
  advertise leftArmCancelName cancels



-- FUNCTIONS
makeCancel :: a -> GID.GoalID
makeCancel _ = (def :: GID.GoalID)

brokenToDirection :: Bool -> Direction
brokenToDirection True = DirBackward
brokenToDirection False = DirForward

--Make sure the position is inside the workspace of the robot
clampPosition :: POI.Point -> POI.Point
clampPosition point@POI.Point{POI.x = x} = point{POI.x = clamp 0.5 0.75 x}

positionsToGoals :: MTP.Arm -> QUA.Quaternion -> POI.Point -> MoveArmActionGoal
positionsToGoals arm orientation position= MTP.makeMoveArmActionGoal $ (POS.Pose position orientation, arm)

delta :: Double
--delta = 0.01
delta = 0.50

directionsToPoints :: Direction -> POI.Point
directionsToPoints DirForward = POI.Point delta 0 0
directionsToPoints DirBackward = POI.Point (-delta) 0 0

sensorToDirectinos :: OB.OpticalBeams -> Direction
sensorToDirectinos beams = if anyBroken beams then DirBackward else DirForward

makeGoal :: OB.OpticalBeams -> MoveArmActionGoal
makeGoal beams = MTP.makeMoveArmActionGoal $ if anyBroken beams then (lArmIn, MTP.LeftArm) else (lArmOut, MTP.LeftArm)

-- UTILITY FUNCTIONS
anyBroken :: OB.OpticalBeams -> Bool
anyBroken = VEC.or . (OB.broken)

integrate :: (Monad m, Monoid a) => a -> Topic m a -> Topic m a
integrate = TOP.scan mappend

integrateWithBounds :: (Monad m, Monoid a) => (a -> a) -> a -> Topic m a -> Topic m a
integrateWithBounds clampFunc initVal = TOP.scan (\ x y -> clampFunc $ mappend  x y) initVal

-- clamps the value between bottom and top
-- if bottom > top then return bottom
clamp :: Ord a => a -> a -> a -> a
clamp bottom top value = max bottom $ min top value
