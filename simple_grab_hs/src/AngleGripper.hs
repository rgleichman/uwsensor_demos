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
import Ros.Pr2_controllers_msgs.JointTrajectoryControllerState as JTS
import qualified Ros.Trajectory_msgs.JointTrajectoryPoint as JTP
import Ros.Node (runHandler)

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

currentArm :: MTP.Arm
currentArm = MTP.LeftArm

statusTopicName :: String
statusTopicName = case currentArm of
  MTP.LeftArm -> leftStatusTopicName
  MTP.RightArm -> rightStatusTopicName

sensorTopicName :: String
sensorTopicName = case currentArm of
  MTP.LeftArm -> leftSensorTopicName
  MTP.RightArm -> rightSensorTopicName

moveArmGoalTopicName = case currentArm of
  MTP.LeftArm -> leftMoveArmGoalTopicName
  MTP.RightArm -> rightMoveArmGoalTopicName

armStateTopicName = case currentArm of
  MTP.LeftArm -> "/l_arm_controller/state"
  MTP.RightArm -> "/r_arm_controller/state"

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

armInPoint = case currentArm of
  MTP.LeftArm -> lArmInPoint

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
main = runNode "AngleGripper" $ goToBlockedState
          

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
goToBlockedState

Behaviorlly:
*Go forwards a little bit at a time if the sensor is not blocked
*If during a step the sensor becomes blocked
*Then record the current joint state
*And command the robot to go to that joint state

Code:
*get sensor messages
*if sensors are not blocked and no active goals, then create a pose goal message
*combine the sensor messages with the joint state messages
*filter the combined messages to be only on transitions from not blocked to blocked
*for each filtered message create a joint goal message from the joint state message
*merge and publish the pose goal topic and the joint goal topic
-}

-- TODO: cycle the integration so that if it goes above the maximum value, it becomes the minimum value
goToBlockedState :: Node ()
goToBlockedState = do
  status <- subscribe statusTopicName
  sensor <- subscribe sensorTopicName
  armState <- subscribe armStateTopicName
  let
    broken = fmap anyBroken sensor
    limitedBroken = MTP.filterNoActive status broken
    directions = fmap brokenToDirection limitedBroken
    onlyForward = TOP.filter (\x -> case x of DirForward -> True
                                              _ -> False)
                  directions
    pointVectors = fmap directionsToPoints onlyForward
    position = fmap unMPoint $ integrateWithBounds
                 (MPoint . clampPosition . unMPoint)
                 (MPoint armInPoint)
                 (fmap MPoint pointVectors)
    goals = fmap (positionsToGoals currentArm MTP.vertical) position
    -- blocked detection
    consecBroken = TU.bothNew (TU.consecutive broken) armState
    breaking = TOP.filter (\(x, _) -> case x of (False, True) -> True
                                                _ -> False) consecBroken
    armStateWhenBroken = fmap snd breaking
    breakingGoals = fmap (armStateToGoal currentArm) armStateWhenBroken
    limitedBreakingGoals = TU.gate breakingGoals limitedBroken
    allGoals = TU.merge goals limitedBreakingGoals
    --allGoals = TU.merge goals breakingGoals
  advertise moveArmGoalTopicName allGoals --(topicRate 10 allGoals)
  advertise "chatter" limitedBreakingGoals


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
positionsToGoals arm orientation position= MTP.makeMoveArmActionGoal $(MTP.PoseGoal $ POS.Pose position orientation, arm)

delta :: Double
delta = 0.2
--delta = 0.50

directionsToPoints :: Direction -> POI.Point
directionsToPoints DirForward = POI.Point delta 0 0
directionsToPoints DirBackward = POI.Point (-delta) 0 0

sensorToDirectinos :: OB.OpticalBeams -> Direction
sensorToDirectinos beams = if anyBroken beams then DirBackward else DirForward

makeGoal :: OB.OpticalBeams -> MoveArmActionGoal
makeGoal beams = MTP.makeMoveArmActionGoal $ if anyBroken beams then (MTP.PoseGoal lArmIn, MTP.LeftArm) else (MTP.PoseGoal lArmOut, MTP.LeftArm)

armStateToGoal :: MTP.Arm -> JTS.JointTrajectoryControllerState -> MoveArmActionGoal
armStateToGoal arm state = MTP.makeMoveArmActionGoal (MTP.JointGoal joints, arm)
  where joints = VEC.toList $ JTP.positions $ JTS.actual state

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
