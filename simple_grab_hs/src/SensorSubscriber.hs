module SensorSubscriber (main) where

import Ros.Node (subscribe)
import Ros.Node (runHandler)
import Ros.Node (runNode)
import Ros.Pr2_pretouch_sensor_optical.OpticalBeams
import Data.Vector.Storable ((!))

rightTopicName :: String
rightTopicName = "optical/right"

leftTopicName :: String
leftTopicName = "optical/left"

showMsg :: OpticalBeams -> IO ()
showMsg  = putStrLn . ("I heard ya " ++ ) . show -- . getFirst
  where
    getFirst =(!0) .  broken
              -- :: OpticalBeams -> Bool

main :: IO ()
main = runNode "SensorSubscriber" $ runHandler showMsg =<< subscribe rightTopicName
