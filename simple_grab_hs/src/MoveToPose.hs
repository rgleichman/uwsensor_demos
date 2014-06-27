module MoveToPose (main) where
import Ros.Node (Topic)
import Ros.Node (topicRate)
import Ros.Node (runNode)
import Ros.Node (advertise)
import Ros.Topic (repeatM)
import Ros.Arm_navigation_msgs.MoveArmActionGoal
import Ros.Arm_navigation_msgs.MoveArmGoal
import qualified Ros.Arm_navigation_msgs.SimplePoseConstraint as SPC
import qualified Ros.Arm_navigation_msgs.MotionPlanRequest as MPR
import qualified Ros.Arm_navigation_msgs.Constraints as CON
import Data.Default.Generics (def)
import qualified Ros.Arm_navigation_msgs.OrientationConstraint as ORC
import qualified Ros.Arm_navigation_msgs.PositionConstraint as POC
import qualified Ros.Geometry_msgs.Pose as POS
import qualified Ros.Arm_navigation_msgs.Shape as SH
import Control.Applicative ((<*>))
import qualified Ros.Geometry_msgs.Point as POI
import Data.Vector.Storable (fromList)
import qualified Ros.Geometry_msgs.Quaternion as QUA
import qualified Ros.Std_msgs.Header as HEA

moveToPose :: Topic IO MoveArmActionGoal
moveToPose = repeatM msg
  where msg = return $ (def :: MoveArmActionGoal){
          goal = makeMoveArmGoal
          }
        
makeMoveArmGoal :: MoveArmGoal
makeMoveArmGoal = (def :: MoveArmGoal) {
  planner_service_name = "ompl_planning/plan_kinematic_path"
  --, planning_scene_diff =
  , motion_plan_request = makeMotionPlanRequest
  --, operations = undefined
  , accept_partial_plans = False
  , accept_invalid_goals = False
  , disable_ik = False
  , disable_collision_monitoring = False
  }

makeMotionPlanRequest :: MPR.MotionPlanRequest
makeMotionPlanRequest =
  (def :: MPR.MotionPlanRequest) {
    MPR.num_planning_attempts = 1
                                -- |ROSDuration is a tuple of (seconds, nanoseconds)
    ,MPR.group_name = "right_arm"
    ,MPR.allowed_planning_time = (1,0)
    , MPR.goal_constraints = goalConstraints
    }
    where
      pose = (def :: SPC.SimplePoseConstraint){
        SPC.header = (def :: HEA.Header) {HEA.frame_id = "torso_lift_link"}
        ,SPC.link_name = "r_wrist_roll_link"
        ,SPC.pose = POS.Pose{
          POS.position = POI.Point{
             POI.x = 0.75
             --POI.x = 0.745
             --POI.x = 0.5
             ,POI.y = -0.188
             ,POI.z = 0
                      --,POI.z = -0.3
             }
          ,POS.orientation = QUA.Quaternion{
             QUA.x = 1.0
             ,QUA.y = 0.0
             ,QUA.z = 0.0
             ,QUA.w = 1.0
             }
          }
        ,SPC.absolute_position_tolerance =  POI.Point{
          POI.x = 0.001
          ,POI.y = 0.001
          ,POI.z = 0.001
          }
        ,SPC.absolute_roll_tolerance = 0.04
        ,SPC.absolute_pitch_tolerance = 0.04
        ,SPC.absolute_yaw_tolerance = 0.04
        }
      goalConstraints = makeGoalConstraints pose
    
makeGoalConstraints :: SPC.SimplePoseConstraint -> CON.Constraints
makeGoalConstraints pose = (def :: CON.Constraints){
  CON.position_constraints = [posConstraint]
  ,CON.orientation_constraints = [orientConstraint]
  }
  where
    (posConstraint, orientConstraint) = makePoseOrientConstraints pose

makePoseOrientConstraints :: SPC.SimplePoseConstraint -> (POC.PositionConstraint, ORC.OrientationConstraint)
makePoseOrientConstraints poseC @SPC.SimplePoseConstraint{SPC.pose = pose,
                                                        SPC.header = header',
                                                        SPC.link_name = linkName,
                                                        SPC.absolute_position_tolerance = absPosTol,
                                                        SPC.orientation_constraint_type = orType
                                                       }
  = (posC, orC)
  where
    posC = (def :: POC.PositionConstraint){
      POC.header = header'
      ,POC.link_name = linkName
      ,POC.position = POS.position $ pose
      ,POC.constraint_region_shape = constraintRegionShape
      ,POC.constraint_region_orientation = constraintRegionOrientation
      ,POC.weight = 1.0
      }
      where
        constraintRegionShape = (def :: SH.Shape) {
          SH._type = SH.bOX
          , SH.dimensions = fromList . map (*2) $ [POI.x, POI.y, POI.z] <*> [absPosTol]
          }
        constraintRegionOrientation = QUA.Quaternion {
          QUA.x = 0
          , QUA.y = 0
          , QUA.z = 0
          , QUA.w = 1.0
          }
    orC = (def :: ORC.OrientationConstraint){
      ORC.header = header'
      , ORC.link_name = linkName
      ,ORC.orientation = POS.orientation $ pose
      ,ORC._type = orType
      ,ORC.absolute_roll_tolerance = SPC.absolute_roll_tolerance poseC
      ,ORC.absolute_pitch_tolerance = SPC.absolute_pitch_tolerance poseC
      ,ORC.absolute_yaw_tolerance = SPC.absolute_yaw_tolerance poseC
      ,ORC.weight = 1.0
      }


main :: IO ()
main = runNode "MoveToPose" $ advertise topicName (topicRate 0.5 moveToPose)
  where topicName = "/move_right_arm/goal"
