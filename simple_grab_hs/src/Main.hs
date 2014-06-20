module Main (main) where
import Ros.Node
import Simple_grab_hs

main = runNode "simple_grab_hs" simple_grab_hs
