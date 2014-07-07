import Ros.Node
import qualified Ros.Topic as TOP
import qualified Ros.Pr2_pretouch_sensor_optical.OpticalBeams as OB
import Control.Applicative
import Data.Default.Generics (def)
import Data.Vector.Storable (fromList)

main :: IO ()
main = do
  runNode "SensorPublisher"
    $ advertise "/optical/right"
    (topicRate (100) $ fmap boolToSensor falseThenTrue)

flipFlop :: (Applicative m) => Topic m Bool
flipFlop = TOP.unfold (\x -> pure $ (x, not x)) False

allFalse :: (Monad m) =>  Topic m Bool
allFalse = TOP.repeatM $ return False

falseThenTrue :: (Applicative m) =>  Topic m Bool
falseThenTrue = TOP.unfold(\n -> pure $ (if n < 200 then False else True, n+1)) 0

boolToSensor :: Bool -> OB.OpticalBeams
boolToSensor b = (def :: OB.OpticalBeams){
  OB.broken = fromList [b, b, b, b]
  }
