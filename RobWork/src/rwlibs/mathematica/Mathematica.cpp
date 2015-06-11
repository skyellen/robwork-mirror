/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "Mathematica.hpp"

// Packets
#include "EnterExpressionPacket.hpp"
#include "EnterTextPacket.hpp"
#include "EvaluatePacket.hpp"
#include "InputNamePacket.hpp"
#include "MessagePacket.hpp"
#include "OutputNamePacket.hpp"
#include "ReturnExpressionPacket.hpp"
#include "ReturnPacket.hpp"
#include "ReturnTextPacket.hpp"
#include "TextPacket.hpp"

// Functions
#include "ToExpression.hpp"
#include "List.hpp"
#include "RawArray.hpp"

#include "wstp.h"

#include <boost/foreach.hpp>

using namespace rw::common;
using namespace rwlibs::mathematica;

struct Mathematica::Environment {
	Environment():
		ep(WSInitialize((WSParametersPointer)0))
	{
	}
	~Environment() {
		if(valid()) {
			WSDeinitialize(ep);
		}
	}
	bool valid() const {
		return ep != 0;
	}
	const WSENV ep;
};

struct Mathematica::LinkImpl {
	typedef rw::common::Ptr<const LinkImpl> Ptr;
	LinkImpl(const Environment* env, const std::string& args):
		env(env),
		err(0)
	{
#if WSINTERFACE >= 3
		lp = WSOpenString(env->ep, args.c_str(), &err);
#else
		lp = WSOpenArgv(env->ep, args.c_str(), &err);
#endif
	}
	~LinkImpl() {
	}
	bool open() const {
		return lp != NULL;
	}
	void close() const {
		if(open()) {
			WSPutFunction(lp, "Exit", 0L);
			WSClose(lp);
		}
	}

private:
	const Environment* env;
#if WSINTERFACE >= 3
	int err;
#else
	long err;
#endif

public:
	WSLINK lp;
};

Mathematica::Link::Link():
	impl(NULL)
{
}

Mathematica::Link::~Link()
{
}

bool Mathematica::Link::isOpen() const {
	if (impl.isNull())
		return false;
	else
		return impl->open();
}

bool Mathematica::Link::ready() const {
	if (link == NULL)
		RW_THROW("The given link was NULL!");
	if (!isOpen())
		RW_THROW("Mathematica link is not open!");
	return WSReady(impl->lp);
}

bool Mathematica::Link::wait() const {
	if (link == NULL)
		RW_THROW("The given link was NULL!");
	if (!isOpen())
		RW_THROW("Mathematica link is not open!");
	return WSWaitForLinkActivity(impl->lp) == WSWAITSUCCESS;
}

const Mathematica::Link& Mathematica::Link::operator<<(const Packet& packet) const {
	if (link == NULL)
		RW_THROW("The given link was NULL!");
	if (!isOpen())
		RW_THROW("Mathematica link is not open!");
	const WSLINK& lp = impl->lp;

	//WSPutFunction(lp, "EvaluatePacket", 1L);
	//put(impl,expression);
	put(impl,packet);
	WSEndPacket(lp);
	return *this;
}

const Mathematica::Link& Mathematica::Link::operator<<(const Expression& expression) const {
	return (*this) << EvaluatePacket(expression);
}

const Mathematica::Link& Mathematica::Link::operator<<(const ToExpression& expression) const {
	return (*this) << EvaluatePacket(expression);
}

void Mathematica::Link::operator>>(rw::common::Ptr<Packet>& result) const {
	result = NULL;
	wait();

	if (link == NULL)
		RW_THROW("The given link was NULL!");
	if (!isOpen())
		RW_THROW("Mathematica link is not open!");
	const WSLINK& lp = impl->lp;

	int pkt = WSNextPacket(lp);
	Mathematica::Expression::Ptr exp = NULL;
	if (pkt) {
		switch(pkt) {
		case ENTEREXPRPKT:
			result = ownedPtr(new EnterExpressionPacket(readExpression(impl)));
			break;
		case ENTERTEXTPKT:
			result = ownedPtr(new EnterTextPacket(readString(impl,false)));
			break;
		case EVALUATEPKT:
			result = ownedPtr(new EvaluatePacket(readExpression(impl)));
			break;
		case INPUTNAMEPKT: // In[n]:=
			result = ownedPtr(new InputNamePacket(readString(impl,false)));
			break;
		case MESSAGEPKT: // Symbol::tag
		{
			const std::string symbol = readString(impl,true);
			const std::string tag = readString(impl,false);
			result = ownedPtr(new MessagePacket(symbol,tag));
			break;
		}
		case OUTPUTNAMEPKT:
			result = ownedPtr(new OutputNamePacket(readString(impl,false)));
			break;
		case RETURNEXPRPKT:
			result = ownedPtr(new ReturnExpressionPacket(readExpression(impl)));
			break;
		case RETURNPKT:
			result = ownedPtr(new ReturnPacket(readExpression(impl)));
			break;
		case RETURNTEXTPKT:
			result = ownedPtr(new ReturnTextPacket(readString(impl,false)));
			break;
		case TEXTPKT:
			result = ownedPtr(new TextPacket(readString(impl,false)));
			break;
		default:
			RW_THROW("Got unknown packet type: " << pkt);
			WSNewPacket(lp);
			break;
		}
		if( WSError(lp)) {
			error(impl);
		}
	}
	if (result == NULL)
		RW_THROW("Did not get any packet.");
}

Mathematica::Mathematica():
	_env(NULL)
{
}

Mathematica::~Mathematica() {
	finalize();
}

bool Mathematica::initialize() {
	if (_env != NULL)
		return false;
	_env = new Environment();
	if(!_env->valid()) {
		delete _env;
		return false;
	}
	return true;
}

Mathematica::Link::Ptr Mathematica::createLink(const std::string& name, LinkProtocol protocol) {
	if (_env == NULL)
		return NULL;

	std::stringstream args;
	args << "-linkcreate";
	if (name != "")
		args << " -linkname \"" + name + "\"";
	args << " -linkprotocol ";
	if (protocol == SharedMemory) {
		args << "SharedMemory";
	} else if (protocol == TCP_IP) {
		args << "TCPIP";
	}

	const rw::common::Ptr<Link> link = ownedPtr(new Link());
	link->name = name;
	link->impl = ownedPtr(new LinkImpl(_env, args.str()));
	if (link->isOpen()) {
		_links.push_back(link);
		return link;
	}
	return NULL;
}

Mathematica::Link::Ptr Mathematica::connectToLink(const std::string& name) {
	if (_env == NULL)
		return NULL;
	if (!_env->valid())
		return NULL;

	std::stringstream args;
	args << "-linkconnect";
	if (name != "")
		args << " -linkname \"" + name + "\"";

	const rw::common::Ptr<Link> link = ownedPtr(new Link());
	link->name = name;
	link->impl = ownedPtr(new LinkImpl(_env, args.str()));
	if (link->isOpen()) {
		_links.push_back(link);
		return link;
	}
	return NULL;
}

Mathematica::Link::Ptr Mathematica::launchKernel() {
	if (_env == NULL)
		return NULL;

	const rw::common::Ptr<Link> link = ownedPtr(new Link());
	link->impl = ownedPtr(new LinkImpl(_env, "-linklaunch -linkname 'math -wstp'"));
	if (link->isOpen()) {
		_links.push_back(link);
		return link;
	}
	return NULL;
}

bool Mathematica::closeLink(Link::Ptr link) {
	std::list<rw::common::Ptr<Link> >::iterator it;
	for (it = _links.begin(); it != _links.end(); it++) {
		if (*it == link) {
			const rw::common::Ptr<Link> l = *it;
			l->impl->close();
			l->impl = NULL;
			_links.erase(it);
			return true;
		}
	}
	return false;
}

void Mathematica::finalize() {
	while (_links.size() > 0) {
		closeLink(_links.front());
	}
	if (_env != NULL) {
		delete _env;
		_env = NULL;
	}
}

Mathematica::FunctionBase::FunctionBase(const std::string& name):
	_name(name)
{
}

std::string Mathematica::FunctionBase::getName() const {
	return _name;
}

Mathematica::Expression::Type Mathematica::FunctionBase::getType() const {
	return Expression::Function;
}

void Mathematica::FunctionBase::out(std::ostream& stream) const {
	out(stream,0);
}

void Mathematica::FunctionBase::out(std::ostream& stream, std::size_t indent) const {
	for (std::size_t i = 0; i < indent; i++)
		stream << " ";
	indent += 3;
	stream << getName() << "[";
	const std::list<rw::common::Ptr<const Expression> >& arguments = getArguments();
	BOOST_FOREACH(const rw::common::Ptr<const Expression> arg, arguments) {
		RW_ASSERT(!arg.isNull());
		const rw::common::Ptr<const FunctionBase> fct = arg.cast<const FunctionBase>();
		if (!fct.isNull()) {
			stream << '\n';
			fct->out(stream,indent);
		} else {
			arg->out(stream);
		}
		if (!(arg == arguments.back()))
			stream << ", ";
	}
	stream << "]";
}

Mathematica::Function::Function(const std::string& name):
	FunctionBase(name)
{
}

void Mathematica::Function::setName(const std::string &name) {
	_name = name;
}

void Mathematica::Function::addArgument(const AutoExpression& arg) {
	_arguments.push_back(arg.expression());
}

void Mathematica::Function::addArgument(const Expression::Ptr arg) {
	_arguments.push_back(arg);
}

void Mathematica::Function::addArguments(const std::list<Expression::Ptr>& args) {
	BOOST_FOREACH(const Expression::Ptr arg, args) {
		addArgument(*arg);
	}
}

void Mathematica::Function::setArgument(std::size_t i, Expression::Ptr arg) {
	std::list<Expression::Ptr>::iterator it = _arguments.begin();
	if (it == _arguments.end())
		RW_THROW("This index does not exist.");
	for (std::size_t id = 0; id < i; id++) {
		it++;
		if (it == _arguments.end())
			RW_THROW("This index does not exist.");
	}
	it = _arguments.erase(it);
	_arguments.insert(it,arg);
}

std::list<rw::common::Ptr<const Mathematica::Expression> > Mathematica::Function::getArguments() const {
	std::list<rw::common::Ptr<const Mathematica::Expression> > res;
	BOOST_FOREACH(const rw::common::Ptr<const Mathematica::Expression> arg, _arguments) {
		res.push_back(arg);
	}
	return res;
}

std::list<Mathematica::Expression::Ptr> Mathematica::Function::getArguments() {
	return _arguments;
}

Mathematica::Expression::Ptr Mathematica::Function::clone() const {
	const Function::Ptr fct = ownedPtr(new Function(_name));
	fct->addArguments(_arguments);
	return fct;
}

/*
rw::common::Ptr<const Mathematica::Expression> Mathematica::FunctionBase::getArgument(std::size_t i) const {
	std::list<Expression::Ptr>::const_iterator it = _arguments.begin();
	if (it == _arguments.end())
		RW_THROW("This index does not exist.");
	for (std::size_t id = 0; id < i; id++) {
		it++;
		if (it == _arguments.end())
			RW_THROW("This index does not exist.");
	}
	return *it;
}*/

void Mathematica::put(LinkImpl::Ptr link, const Expression& expression) {
	const WSLINK& lp = link->lp;
	switch(expression.getType()) {
	case Expression::Function:
	{
		const FunctionBase& fct = dynamic_cast<const FunctionBase&>(expression);
		const std::list<rw::common::Ptr<const Expression> >& args = fct.getArguments();
		WSPutFunction(lp, fct.getName().c_str(), (int) args.size());
		BOOST_FOREACH(const rw::common::Ptr<const Expression> arg, args) {
			put(link, *arg);
		}
	}
	break;
	case Expression::Integer:
	{
		const Integer& e = dynamic_cast<const Integer&>(expression);
		WSPutInteger(lp, e.value());
	}
	break;
	case Expression::Real:
	{
		const Real& e = dynamic_cast<const Real&>(expression);
		WSPutReal(lp, e.value());
	}
	break;
	case Expression::String:
	{
		const String& e = dynamic_cast<const String&>(expression);
		WSPutString(lp, e.value().c_str());
	}
	break;
	case Expression::Symbol:
	{
		const Symbol& e = dynamic_cast<const Symbol&>(expression);
		WSPutSymbol(lp, e.getName().c_str());
	}
	break;
	case Expression::Array:
	{
		try {
			const Array<unsigned char>& e = dynamic_cast<const Array<unsigned char>&>(expression);
			const unsigned char* data = e.data();
			const int depth = e.dimensions();
			const int* dims = e.size();
			if(!WSPutInteger8Array(link->lp, data, dims, NULL, depth)) {
				RW_THROW("Could not write byte array");
			}
		} catch(const std::bad_cast&) {
			RW_THROW("The given array type is not yet supported.");
		}
	}
	break;
	}
}

std::string Mathematica::readString(LinkImpl::Ptr link, bool symbol) {
	const int type = symbol? WSTKSYM : WSTKSTR;
#if WSINTERFACE >= 3
	const char *s;
#else
	kcharp_ct  s;
#endif /* WSINTERFACE >= 3 */
	std::string res;
	if (WSGetNext(link->lp) == type) {
		WSGetString(link->lp, &s);
		res = s;
#if WSINTERFACE >= 4
		WSReleaseString(link->lp, s);
#else
		WSDisownString(link->lp, s);
#endif
	} else {
		if (symbol)
			RW_THROW("Expected symbol");
		else
			RW_THROW("Expected string");
	}
	return res;
}

Mathematica::Expression::Ptr Mathematica::readExpression(LinkImpl::Ptr link) {
	const Function::Ptr wrap = ownedPtr(new Function("Wrap"));
	addExpression(wrap,link);
	return wrap->getArguments().front();
}

void Mathematica::addExpression(Function::Ptr exp, LinkImpl::Ptr link) {
#if WSINTERFACE >= 3
	const char *s;
#else
	kcharp_ct  s;
#endif /* WSINTERFACE >= 3 */
	std::string str;
	int n;
	int i, len;
	double r;

	switch( WSGetNext(link->lp)) {
	case WSTKSYM:
		WSGetSymbol(link->lp, &s);
		str = s;
#if WSINTERFACE >= 4
		WSReleaseSymbol(link->lp, s);
#else
		WSDisownSymbol(link->lp, s);
#endif
		exp->addArgument(ownedPtr(new Symbol(str)));
		break;
	case WSTKSTR:
		WSGetString(link->lp, &s);
		str = s;
#if WSINTERFACE >= 4
		WSReleaseString(link->lp, s);
#else
		WSDisownString(link->lp, s);
#endif
		exp->addArgument(ownedPtr(new String(str)));
		break;
	case WSTKINT:
		WSGetInteger(link->lp, &n);
		exp->addArgument(ownedPtr(new Integer(n)));
		break;
	case WSTKREAL:
		WSGetReal(link->lp, &r);
		exp->addArgument(ownedPtr(new Real(r)));
		break;
	case WSTKFUNC:
		if( WSGetArgCount(link->lp, &len) == 0){
			error(link);
		} else {
			const std::string fctName = readString(link,true);
			if (fctName == "RawArray") {
				const std::string type = readString(link,false);
				if (type == "Byte") {
					unsigned char* data;
					int* dims;
					int depth;
					char** heads;
					if(!WSGetInteger8Array(link->lp, &data, &dims, &heads, &depth)) {
						RW_THROW("Could not read byte array");
					}
					RW_ASSERT(depth == 3);
					const Array<unsigned char>::Ptr array = ownedPtr(new RawArray<unsigned char,Dynamic>(data,dims,depth));
					WSReleaseInteger8Array(link->lp, data, dims, heads, depth);
					exp->addArgument(array);
				} else {
					RW_THROW("RawArray of type " << type << " can not yet be handled!");
				}
			} else {
				Function::Ptr const fct = ownedPtr(new Function(fctName));
				for( i = 1; i <= len; ++i){
					addExpression(fct,link);
				}
				exp->addArgument(fct);
			}
		}
		break;
	case WSTKERROR:
	default:
		if(WSError(link->lp)) {
			RW_THROW(WSErrorMessage(link->lp));
		} else {
			RW_THROW("Error detected by this program.");
		}
		break;
	}
}

int Mathematica::expectFunction(LinkImpl::Ptr link, const std::string& name) {
	int len = 0;
	if (WSGetNext(link->lp) == WSTKFUNC) {
		if( WSGetArgCount(link->lp, &len) == 0) {
			error(link);
		} else {
			const std::string fctName = readString(link,true);
			if (fctName != name)
				RW_THROW("Expected function of name \"" << name << "\" - not \"" << fctName << "\".");
			len--;
		}
	} else {
		RW_THROW("Expected function!");
	}
	return len;
}

void Mathematica::error(LinkImpl::Ptr link) {
	if(WSError(link->lp)) {
		RW_THROW("Error detected by WSTP: " << WSErrorMessage(link->lp) << ".");
	} else {
		RW_THROW("Error detected by this program.");
	}
}

std::ostream& rwlibs::mathematica::operator<<(std::ostream& out, const Mathematica::Expression& expression) {
	expression.out(out);
	/*try {
		const Mathematica::FunctionBase& fct = dynamic_cast<const Mathematica::FunctionBase&>(expression);
		fct.out(out,0);
	} catch(const std::bad_cast&) {
		expression.out(out);
	}*/
	return out;
}

Mathematica::AutoExpression::AutoExpression(const std::list<AutoExpression>& args):
	_exp(rw::common::ownedPtr(new List()))
{
	BOOST_FOREACH(const AutoExpression& val, args) {
		_exp.cast<List>()->add(val);
	}
}

#if __cplusplus >= 201103L
Mathematica::AutoExpression::AutoExpression(const std::initializer_list<AutoExpression>& args):
	_exp(rw::common::ownedPtr(new List()))
{
	for(const AutoExpression& val : args) {
		_exp.cast<List>()->add(val);
	}
}
#endif
