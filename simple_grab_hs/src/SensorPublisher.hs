import Ros.Node
import qualified Ros.Topic as TOP
import qualified Ros.Pr2_pretouch_sensor_optical.OpticalBeams as OB
import Control.Applicative
import Data.Default.Generics (def)
import Data.Vector.Storable (fromList)

main :: IO ()
main = do
  runNode "SensorPublisher" $ advertise "/optical/left" (topicRate (5) $ fmap boolToSensor flipFlop)

flipFlop :: (Applicative m) => Topic m Bool
flipFlop = TOP.unfold (\x -> pure $ (x, not x)) True

boolToSensor :: Bool -> OB.OpticalBeams
boolToSensor b = (def :: OB.OpticalBeams){
  OB.broken = fromList [b, b, b, b]
  }
