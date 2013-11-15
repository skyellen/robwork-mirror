pos = rw.Vector3D(0.0404794,-3.69926e-05, 0.0489482)
rot = rw.Quaternion(-0.831552,-0.0553467,-0.160624, 0.528825):toRotation3D()

pos = rw.Vector3D(0.0399972, 0.000177004, 0.0481545)
rot = rw.Quaternion(-0.833686, -0.0512892, -0.173265, 0.52184):toRotation3D()

wTtcp_target = rw.Transform3D(pos, rot)

print( wTbase_target ) 
mframe = rws.findMovableFrame("GS20.Base")
tcpframe = rws.findFrame("TCPGS20")

wTbase = rw.worldTframe(mframe, rws.getState() )
wTtcp = rw.worldTframe(tcpframe, rws.getState() )
baseTtcp = wTbase:inverse() * wTtcp

wTbase_target = wTtcp_target * baseTtcp:inverse()

state = rws.getState()

mframe:setTransform(wTbase_target, state)

rws.setState(state)