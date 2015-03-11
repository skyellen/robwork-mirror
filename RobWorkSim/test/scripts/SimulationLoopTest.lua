--
-- scene should have two bodies one fixed (FBody) and one RigidBody (RBody) that are constrained by a motorized joint
-- 
--   
--

dwc = rwsim.loadDWC("FTSensor_test.dwc.xml")

simulator = rwsim.DynamicSimulator(dwc)
simthread = rwsim.SimulatorThread(simulator)

rbody = dwc:getRigidBody("RBody")
ftsensor = dwc:getFTSensor("FTSensor")
simulator:setForceTarget( rbody, rw.Vector3D(1,0,0), rw.Vector3D(0,0,0) )

simthread.start()

-- now log how the sensor behave under varying forces

result = {} -- new array
for i=1, 1000 do
  state = simthread:getState()
  time = simthread:getTime()
  force = ftsensor:getForce(state)
  torque = ftsensor:getTorque(state)
  -- TODO save the values in a list
  a[i] = {time, force, torque}
  rw.sleep(0.1)
end

-- todo analyse the results
for i=1, 1000 do
  {time, force, torque} = a[i]
  print(time .. "\t" .. force[0] .. "\t" .. force[1] .. "\t" .. force[2]) 
  print("\t" .. torque[0] .. "\t" .. torque[1] .. "\t" .. torque[2])
  --todo analyse the results
end


