-- get all handles
dwc = rwsim.getDynamicWorkCell()
sim = rwsim.getSimulatorInstance()
state = rwstudio:getState()
sctrl = dwc:findSerialDeviceController("URController")
ft = dwc:findFTSensor("FTSensor")



-- now give a FT command 
taskframe = "UR-6-85-5-A"
-- the selection 1 means force control, 0 means position control
selection = rw.Q(6,0,0,0,0,0,0)
-- target transform
t3d_target = rw.Transform3D( rw.Vector3D(0.609, -0.131,0.05), rw.RPY(-1.608,0,-2.790):toRotation3D() )
-- target wrench, push with 10N in the direction of z-axis
wrench_target = rw.Wrench6D(0,0,10,0,0,0)
-- offset relative to refframe
t3d_offset = rw.Transform3D( rw.Vector3D(0, 0,0), rw.RPY(-0,0,0):toRotation3D() )
-- move
sctrl:moveLinFC(t3d_target,wrench_target,selection,"UR-6-85-5-A", t3d_offset, 100, 0)
