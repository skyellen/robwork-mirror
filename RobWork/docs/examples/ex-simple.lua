
function b2s(b)
    if b then
        return "true"
    else
        return "false"
    end
end

x = 2.92
print("Radians to degrees and back:", b2s(rw.d2r(rw.r2d(x)) == x))
print()

print("rpy is", rw.rpy(0.3, 0, 0))
print("rpy is", rw.rpy_deg(rw.r2d(0.3), 0, 0))
print("eaa is", rw.eaa(0.3, 0, 0))
print("eaa is", rw.eaa_deg(rw.r2d(0.3), 0, 0))
print()

q = rw.Q({1, 2, 3, 4, 5, 6, 7})
print("q is", q)
print()

v = rw.Vector3D({0, 3, -0.3})
print("v is", v)
print("v * 2 is", v * 2)
print("v + v - v is v:", b2s(v + v - v == v))
print()

r = rw.Rotation3D({1, 0, 0, 0, 1, 0, 0, 0, 1})
print("r is", r)
print("r^2 is", r * r)
print("rw.inverse(r) is", rw.inverse(r))
print("r:inverse() is", r:inverse())
print("The type of Rotation3D is", tolua.type(r))
print()

t = rw.Transform3D(v, r)
print("t is", t)
print("rw.inverse(t) is", rw.inverse(t))
print("t:inverse() is", t:inverse())
print("The rotation of t is", t:r())
print("The translation of t is v:", b2s(v == t:p()))
print()

print("r * v is", r * v)
print("(t * t) * v is", (t * t) * v)
print()

workcell = rw.loadWorkCell("workcell.wu")
print("Workcell is", workcell)

state = workcell:getDefaultState()

frame = workcell:getFrame("Device.TCP")
print("Frame is", frame)

world = workcell:getWorldFrame()
print("World frame is", world)
print("World to frame transform is", world:to(frame, state))
print("Frame world transform is", frame:wt(state))
print("Frame world translation is",
      frame:wt(state):p() - world:wt(state):p())
print()

device = workcell:getDevice("Device")
print("Device is", device)
print("Device base is", device:getBase())
print("Device end is", device:getEnd())
print("Device configuration is", device:getQ(state))
print()

q = rw.Q{ 0.98, 1.503, -1.521, -0.76, -0.237, -0.895, 0}
print("Assigning device configuration to state.")
device:setQ(q, state)
print("Device configuration is assigned configuration:", b2s(device:getQ(state) == q))
print()

item = workcell:getFrame("Item")
pos = item:wt(state):p()
print("Item position is", pos)
rw.gripFrame(item, device:getEnd(), state)
print("Item position is", item:wt(state):p())
item:attachFrame(world, state)
print("Item position is", item:wt(state):p())
