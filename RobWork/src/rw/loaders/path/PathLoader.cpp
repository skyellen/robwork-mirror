/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
 * Faculty of Engineering, University of Southern Denmark 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#include "PathLoader.hpp"

#include <rw/models/Models.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/common/IOUtil.hpp>
#include <rw/trajectory/TimedUtil.hpp>

#include <fstream>

using namespace rw::math;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::trajectory;

namespace
{
    std::string
    quote(const std::string& str)
    {
        return StringUtil::quote(str);
    }

    std::string optionalS(int cnt)
    {
        return cnt == 1 ? "" : "s";
    }

    void readFile(const std::string& file_name, std::vector<char>& result)
    {
        IOUtil::readFile(file_name, result);
    }

    void writeFile(
        const std::vector<char>& input,
        const std::string& file)
    {
        std::ofstream out(file.c_str(), std::ofstream::binary);
        if (!out.is_open())
            RW_THROW("Can't write file " << quote(file));

        if (!input.empty())
            out.write(&input.front(), input.size());
    }

    void attachFrame(State& state, Frame& frame, Frame& parent)
    {
        frame.attachTo(&parent, state);
    }

    const double* getQ(const State& state, const Frame& frame)
    {
        return frame.getData(state);
    }

    void setQ(State& state, const Frame& frame, const double* vals)
    {
        frame.setData(state, vals);
    }

    bool isDAF(const Frame& frame, const State& dummy)
    {
        return
            !frame.getParent() &&
            frame.getDafParent(dummy); // Special case for the world frame needed!
    }

    Frame& findFrame(
        const WorkCell& workcell, const std::string& name)
    {
        Frame* frame = workcell.findFrame(name);
        if (!frame)
            RW_THROW(
                "No frame named "
                << quote(name)
                << " in work cell.");
        return *frame;
    }

    class Reader
    {
    public:
        Reader(
            const std::string& name,
            const std::vector<char>* input)
            :
            _name(name),
            _input(input),
            _pos(0)
        {}

        int getInt()
        {
            const int len = (int)sizeof(int);
            if (!canRead(len))
                die("Integer expected.");

            int x = *(int *)get();
            inc(len);
            return x;
        }

        const char* getCharArray(int n)
        {
            const int len = n;
            if (!canRead(len))
                die("Char array expected.");

            const char* cs = get();
            inc(len);
            return cs;
        }

        double getDouble()
        {
            const int len = (int)sizeof(double);
            if (!canRead(len))
                die("Double expected.");

            const double val = *(double *)get();
            inc(len);
            return val;
        }

        const double* getDoubleArray(int n)
        {
            const int len = n * (int)sizeof(double);
            if (!canRead(len))
                die("Double array expected.");

            const double* vals = (const double*)get();
            inc(len);
            return vals;
        }

        std::string getString()
        {
            const int n = getInt();
            const char* cs = getCharArray(n);
            return std::string(cs, cs + n);
        }

        void getTreeState(
            const WorkCell& workcell,
            State& state)
        {
            const int n = getInt();

            for (int i = 0; i < n; i++) {
                const std::string frame = getString();
                const std::string parent = getString();

                attachFrame(
                    state,
                    findFrame(workcell, frame),
                    findFrame(workcell, parent));
            }
        }

        void getQState(const WorkCell& workcell, State& state)
        {
            // We recompute this value a lot of times: A little slow, yes, but
            // we don't care much right now.
            const std::vector<Frame*> frames = Models::findAllFrames(workcell);

            const int len = getInt();
            const int frameCnt = (int)frames.size();

            if (len != frameCnt)
                RW_THROW(
                    header()
                    << "Incompatible configuration state. "
                    << "Work cell has "
                    << frameCnt
                    << " frame"
                    << optionalS(frameCnt)
                    << ", but state assumes "
                    << len
                    << " frame"
                    << optionalS(len)
                    << ".");

            // Read the configuration for each frame.
            typedef std::vector<Frame*>::const_iterator I;
            for (I p = frames.begin(); p != frames.end(); ++p) {
                const Frame& frame = **p;
                const int n = frame.getDOF();
                setQ(state, frame, getDoubleArray(n));
            }
        }

        State getState(const WorkCell& workcell)
        {
            State state = workcell.getDefaultState();
            getQState(workcell, state);
            getTreeState(workcell, state);
            return state;
        }

        Timed<State> getTimedState(const WorkCell& workcell)
        {
            const double time = getDouble();
            const State state = getState(workcell);
            return Timed<State>(time, state);
        }

        StatePath getStatePath(const WorkCell& workcell)
        {
            StatePath path;
            const int n = getInt();
            for (int i = 0; i < n; i++)
                path.push_back(getState(workcell));

            if (canRead(1))
                RW_THROW(
                    header()
                    << "End of file expected. "
                    << "Number of state points read were "
                    << (int)path.size());

            return path;
        }

        TimedStatePath getTimedStatePath(const WorkCell& workcell)
        {
            const int n = getInt();

            TimedStatePath path;
            for (int i = 0; i < n; i++) {
                path.push_back(getTimedState(workcell));
            }

            if (canRead(1))
                RW_THROW(
                    header()
                    << "End of file expected. "
                    << "Number of state time points read were "
                    << (int)path.size());

            return path;
        }

        QPath getPath()
        {
            const int m = getInt(); //first int is the number of Q's
            const int n = getInt(); //second int is the size of the Q's
            QPath path;
            for (int i = 0; i<m; i++) {
                Q q(n);
                for (int j = 0; j<n; j++) {
                    q(j) = getDouble();
                }
                path.push_back(q);
            }
            return path;
        }

        // We need this so that we can see if we are reading things of length
        // zero...
        int getPos() const { return _pos; }

        bool isEnd() const { return _pos >= (int)size(); }

    private:
        void die(const std::string& msg)
        {
            RW_THROW(
                header()
                << "Unexpected end of file: "
                << msg);
        }

        std::string header() { return "Reading " + quote(_name) + ": "; }

        bool canRead(int len) const { return _pos + len <= (int)size(); }
        void inc(int len) { _pos += len; }

        const char* get() const { return &_input->at(0) + _pos; }
        std::size_t size() const { return _input->size(); }

        std::string _name; // Identifier for the resource being read.
        const std::vector<char>* _input;
        int _pos;
    };

    class Writer
    {
    public:
        Writer(std::vector<char>* output) :
            _output(output)
        {
            RW_ASSERT(output);
        }

        void putInt(int x)
        {
            put((char*)&x, sizeof(int));
        }

        void putCharArray(const char* cs, int len)
        {
            put(cs, len);
        }

        void putDouble(double val)
        {
            put((char*)&val, sizeof(double));
        }

        void putDoubleArray(const double* qs, int len)
        {
            put((const char*)qs, len * sizeof(double));
        }

        void putString(const std::string& str)
        {
            const int len = (int)str.size();
            putInt(len);
            putCharArray(str.c_str(), len);
        }

        void putTreeState(
            const State& state,
            const std::vector<Frame*>& frames)
        {
            // Find out what frames are DAFs.
            std::vector<Frame*> dafs;
            typedef std::vector<Frame*>::const_iterator I;
            for (I p = frames.begin(); p != frames.end(); ++p) {
                Frame* frame = *p;
                if (isDAF(*frame, state))
                    dafs.push_back(frame);
            }

            // Put them.
            putInt((int)dafs.size());
            for (I p = dafs.begin(); p != dafs.end(); ++p) {
                Frame& frame = **p;
                putString(frame.getName());
                RW_ASSERT(frame.getDafParent(state));
                putString(frame.getDafParent(state)->getName());
            }
        }

        void putQState(
            const State& state,
            const std::vector<Frame*>& frames)
        {
            putInt((int)frames.size());

            // For each frame, append its configuration state to result.
            typedef std::vector<Frame*>::const_iterator I;
            for (I p = frames.begin(); p != frames.end(); ++p) {
                const Frame& frame = **p;
                const int len = frame.getDOF();
                const double* qs = getQ(state, frame);
                putDoubleArray(qs, len);
            }
        }

        void putQ(const Q& q) {
            for (size_t i = 0; i<q.size(); i++) {
                putDouble(q(i));
            }
        }

        void putState(const State& state, const std::vector<Frame*>& frames)
        {
            putQState(state, frames);
            putTreeState(state, frames);
        }

        void putTimedState(const Timed<State>& pnt, const std::vector<Frame*>& frames)
        {
            putDouble(pnt.getTime());
            putState(pnt.getValue(), frames);
        }

        void putStatePath(
            const StatePath& path,
            const std::vector<Frame*>& frames)
        {
            putInt((int)path.size());
            typedef StatePath::const_iterator I;
            for (I p = path.begin(); p != path.end(); ++p)
                putState(*p, frames);
        }

        void putTimedStatePath(const TimedStatePath& path,
                               const std::vector<Frame*>& frames)
        {
            putInt((int)path.size());

            typedef TimedStatePath::const_iterator I;
            for (I p = path.begin(); p != path.end(); ++p) {
                putTimedState(*p, frames);
            }
        }

        void putPath(const QPath& path)
        {
            putInt((int)path.size());
            if (path.size() > 0)
                putInt((int)path.front().size());
            else
                putInt((int)0);

            for (QPath::const_iterator it = path.begin(); it != path.end(); ++it)
                putQ(*it);
        }

    private:
        void put(const char* cs, int size)
        {
            _output->insert(_output->end(), cs, cs + size);
        }

    private:
        std::vector<char>* _output;
    };
}

//----------------------------------------------------------------------
// Path

void PathLoader::storePath(const QPath& path, const std::string& file)
{
    std::vector<char> result;
    Writer writer(&result);
    writer.putPath(path);
    writeFile(result, file);
}

QPath PathLoader::loadPath(const std::string& file)
{
    std::vector<char> input;
    readFile(file, input);
    Reader reader(file, &input);
    return reader.getPath();
}

//----------------------------------------------------------------------
// StatePath

void PathLoader::storeStatePath(
    const WorkCell& workcell,
    const StatePath& path,
    const std::string& file)
{
    const std::vector<Frame*> frames = Models::findAllFrames(workcell);

    std::vector<char> result;
    Writer writer(&result);
    writer.putStatePath(path, frames);
    writeFile(result, file);
}

StatePath PathLoader::loadStatePath(
    const WorkCell& workcell,
    const std::string& file)
{
    std::vector<char> input;
    readFile(file, input);
    Reader reader(file, &input);

    return reader.getStatePath(workcell);
}

//----------------------------------------------------------------------
// TimedStatePath

void PathLoader::storeTimedStatePath(
    const WorkCell& workcell,
    const TimedStatePath& path,
    const std::string& file)
{
    const std::vector<Frame*> frames = Models::findAllFrames(workcell);

    std::vector<char> result;
    Writer writer(&result);
    writer.putTimedStatePath(path, frames);
    writeFile(result, file);
}

void PathLoader::storeVelocityTimedStatePath(
    const WorkCell& workcell,
    const StatePath& path,
    const std::string& file)
{
    if (path.empty())
        RW_THROW("Can't store empty state path");

    storeTimedStatePath(
        workcell,
        TimedUtil::makeTimedStatePath(workcell, path),
        file);
}

TimedStatePath PathLoader::loadTimedStatePath(
    const WorkCell& workcell, const std::string& file)
{
    std::vector<char> input;
    readFile(file, input);
    Reader reader(file, &input);

    return reader.getTimedStatePath(workcell);
}
