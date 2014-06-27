module MoveToPose (main) where
import Ros.Node (Topic)
import Ros.Node (topicRate)
import Ros.Node (runNode)
import Ros.Node (advertise)
import Ros.Topic (repeatM)
import Ros.Arm_navigation_msgs.MoveArmActionGoal
import Ros.Arm_navigation_msgs.MoveArmGoal
import qualified Ros.Arm_navigation_msgs.MotionPlanRequest as MPR
import Data.Default.Generics (def)

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
makeMotionPlanRequest = def :: MPR.MotionPlanRequest
  
main :: IO ()
main = runNode "MoveToPose" $ advertise topicName (topicRate (0.5) moveToPose)
  where topicName = "move_right_arm"
