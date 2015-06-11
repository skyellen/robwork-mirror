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

#ifndef RWLIBS_MATHEMATICA_MATHEMATICA_HPP_
#define RWLIBS_MATHEMATICA_MATHEMATICA_HPP_

/**
 * @file Mathematica.hpp
 *
 * \copydoc rwlibs::mathematica::Mathematica
 */

#include <rw/common/Ptr.hpp>
#include <rw/common/macros.hpp>

#include <list>
#include <vector>

namespace rwlibs {
namespace mathematica {

class ToExpression;

//! @addtogroup mathematica

//! @{
/**
 * @brief Implementation of the Wolfram Symbolic Transfer Protocol (WSTP) to allow communication with %Mathematica.
 *
 * Example of basic usage:
 *
 * \code{.cpp}
 *  Mathematica m;
 *  m.initialize();
 *  const Mathematica::Link::Ptr l = m.launchKernel();
 *  Mathematica::Packet::Ptr result;
 *  *l >> result;
 *  *l << "Solve[x^2 + 2 y^3 == 3681 && x > 0 && y > 0, {x, y}, Integers]";
 *  *l >> result;
 *  std::cout << *result << std::endl;
 * \endcode
 *
 * Construction of a %Mathematica environment, m, makes it possible to create WSTP links (Mathematica::Link).
 * Please note that the %Mathematica object automatically closes all links at destruction.
 * WSTP links can be used for different types of interprocess communication with Wolfram Symbolic expressions.
 * In this example we launch and connect to a kernel.
 *
 * The first expression (the In[1]:= prompt) is received from the kernel with the stream operator *l >> result.
 * We simply ignore this value, and send the first command to be evaluated with the stream operator.
 * The result is then retrieved and printed:
 *
 * \verbatim
  ReturnPacket[
    List[
      List[
         Rule[x, 15],
         Rule[y, 12]],
      List[
         Rule[x, 41],
         Rule[y, 10]],
      List[
         Rule[x, 57],
         Rule[y, 6]]]]
   \endverbatim
 *
 * Streaming of a string to the link will implicitly create a ToExpression expression and wrap it in an EvaluatePacket.
 * The kernel will evaluate the expression in the packet and send back a ReturnPacket with the result (as an expression).
 * Many different types of packets can be sent and received on the link (see Mathematica::PacketType). It is up to the user to deal with the different
 * packet types, and to parse the results received.
 */
class Mathematica {
public:
	//! @brief Smart pointer.
	typedef rw::common::Ptr<Mathematica> Ptr;

	//! @brief Available link protocols.
	enum LinkProtocol {
		//! @brief Use shared memory.
		SharedMemory,
		//! @brief Use TCP/IP.
		TCP_IP
	};

	//! @brief Packet types.
	enum PacketType {
		//! @brief EnterExpressionPacket
		EnterExpression,
		//! @brief EnterTextPacket
		EnterText,
		//! @brief EvaluatePacket
		Evaluate,
		//! @brief InputNamePacket
		InputName,
		//! @brief MessagePacket
		Message,
		//! @brief OutputNamePacket
		OutputName,
		//! @brief ReturnExpressionPacket
		ReturnExpression,
		//! @brief ReturnPacket
		Return,
		//! @brief ReturnTextPacket
		ReturnText,
		//! @brief TextPacket
		Text
	};

	//! @brief A representation of an internal WSTP link.
	struct LinkImpl;

	class Expression;

	class Packet;

	//! @brief Representation of a link.
	struct Link {
		//! @brief Smart pointer to a link.
		typedef rw::common::Ptr<const Link> Ptr;

		//! @brief Constructor.
		Link();

		//! @brief Destructor.
		~Link();

		//! @brief Name of the link if name was given.
		std::string name;

		//! @brief Internals.
		rw::common::Ptr<const LinkImpl> impl;

		/**
		 * @brief Check if the link is open.
		 * @return true if open, false otherwise.
		 */
		bool isOpen() const;

		/**
		 * @brief Check if there is anything available on link.
		 * @return true if a packet is available.
		 */
		bool ready() const;

		/**
		 * @brief Wait until packet becomes available.
		 * @return false if error occurred.
		 */
		bool wait() const;

		/**
		 * @brief Send packet on link.
		 * @param packet [in] the packet to send.
		 * @return a reference to the link for chaining.
		 */
		const Link& operator<<(const Packet& packet) const;

		/**
		 * @brief Send an expression on link in a EvaluatePacket.
		 * @param expression [in] the expression to send.
		 * @return a reference to the link for chaining.
		 */
		const Link& operator<<(const Expression& expression) const;

		/**
		 * @brief Send a text string with an expression on link in a EvaluatePacket.
		 * @param expression [in] the expression to send.
		 * @return a reference to the link for chaining.
		 */
		const Link& operator<<(const ToExpression& expression) const;

		/**
		 * @brief Get the next packet on link (blocks until packet is available).
		 * @param result [out] a pointer to the retrieved packet.
		 */
		void operator>>(rw::common::Ptr<Packet>& result) const;
	};

	//! @brief Constructor.
	Mathematica();

	//! @brief Destructor.
	virtual ~Mathematica();

	/**
	 * @brief Initialize the WSTP framework.
	 * @return true if success.
	 */
	bool initialize();

	/**
	 * @brief Create a new link.
	 * @param name [in] (optional) a named link allows other programs to connect to it.
	 * @param protocol [in] (optional) the type of LinkProtocol to use - default is shared memory.
	 * @return the link.
	 */
	Link::Ptr createLink(const std::string& name = "", LinkProtocol protocol = SharedMemory);

	/**
	 * @brief Connect to an existing link.
	 * @param name [in] the name of the link to connect to.
	 * @return a pointer to the link, or NULL if connection failed.
	 */
	Link::Ptr connectToLink(const std::string& name);

	/**
	 * @brief Launch a kernel.
	 * @return link to the kernel.
	 */
	Link::Ptr launchKernel();

	/**
	 * @brief Close a link.
	 * @param link [in] the link to close.
	 * @return true if link was found and closed, false if it was already closed or not found.
	 */
	bool closeLink(Link::Ptr link);

	//! @brief Close all open links, and deinitialize the WSTP framework.
	void finalize();

	class AutoExpression;

	/**
	 * @brief A representation of a Mathematica expression.
	 */
	class Expression {
	public:
		//! @brief Smart pointer type.
		typedef rw::common::Ptr<Expression> Ptr;

		//! @brief Type of expression.
		typedef enum Type {
			//! @brief A string primitive.
			String,
			//! @brief An integer primitive.
			Integer,
			//! @brief A real primitive.
			Real,
			//! @brief A symbol primitive.
			Symbol,
			//! @brief A function.
			Function,
			//! @brief An array.
			Array
		} Type;

		/**
		 * @brief Print to output stream.
		 * @param stream [in/out] the stream to print to.
		 */
		virtual void out(std::ostream& stream) const = 0;

		/**
		 * @brief Get the type of expression.
		 * @return the type.
		 */
		virtual Type getType() const = 0;

		/**
		 * @brief Make a copy of the expression.
		 * @return a new copy.
		 */
		virtual Expression::Ptr clone() const = 0;

#if __cplusplus >= 201103L
	protected:
		/**
		 * @brief Helper function for extracting a list of arguments when given as a variable number of arguments.
		 * @param [in] the list of arguments.
		 * @note Only available for C++11
		 */
		template <typename Type>
		static void toList(std::list<rw::common::Ptr<Type> >&) {}

		/**
		 * @brief Helper function for extracting a list of arguments when given as a variable number of arguments.
		 * @param list [in] the list of arguments.
		 * @param r [in] the first argument.
		 * @param t [in] the rest of the arguments (variable number of arguments)
		 * @note Only available for C++11
		 */
		template <typename Type, typename Exp, typename... T>
		static void toList(std::list<rw::common::Ptr<Type> >& list, const Exp& r, T... t) {
			const rw::common::Ptr<Type> exp = AutoExpression(r).expression().cast<Type>();
			if (exp == NULL)
				RW_THROW("Encountered expression of unexpected type.");
			else
				list.push_back(exp);
			toList(list,t...);
		}
#endif
	};

	//! @brief A string primitive.
	class String: public Expression {
	public:
		//! @brief Smart pointer type
		typedef rw::common::Ptr<String> Ptr;

		/**
		 * @brief Construct new primitive.
		 * @param value [in] the value.
		 */
		String(const char* value): _value(value) {}

		/**
		 * @brief Construct new primitive.
		 * @param value [in] the value.
		 */
		String(const std::string& value): _value(value) {}

		//! @brief Destructor.
		virtual ~String() {}

		//! @copydoc Expression::getType
		Type getType() const { return Expression::String; }

		//! @copydoc Expression::out
		void out(std::ostream& stream) const { stream << _value; }

		//! @copydoc Expression::clone
		virtual Expression::Ptr clone() const { return rw::common::ownedPtr( new String(_value) ); }

		/**
		 * @brief Get the value.
		 * @return the value.
		 */
		const std::string& value() const { return _value; }

	private:
		const std::string _value;
	};

	//! @brief An integer primitive.
	class Integer: public Expression {
	public:
		//! @brief Smart pointer type
		typedef rw::common::Ptr<Integer> Ptr;

		/**
		 * @brief Construct new integer.
		 * @param value [in] the value.
		 */
		Integer(int value): _value(value) {}

		//! @brief Destructor.
		virtual ~Integer() {}

		//! @copydoc Expression::getType
		Type getType() const { return Expression::Integer; }

		//! @copydoc Expression::out
		void out(std::ostream& stream) const { stream << _value; }

		//! @copydoc Expression::clone
		virtual Expression::Ptr clone() const { return rw::common::ownedPtr( new Integer(_value) ); }

		/**
		 * @brief Get the value.
		 * @return the value.
		 */
		int value() const { return _value; }

	private:
		int _value;
	};

	//! @brief A real primitive.
	class Real: public Expression {
	public:
		//! @brief Smart pointer type
		typedef rw::common::Ptr<Real> Ptr;

		/**
		 * @brief Construct new floating point value.
		 * @param value [in] the value.
		 */
		Real(double value): _value(value) {}

		//! @brief Destructor.
		virtual ~Real() {}

		//! @copydoc Expression::getType
		Type getType() const { return Expression::Real; }

		//! @copydoc Expression::out
		void out(std::ostream& stream) const { stream << _value; }

		//! @copydoc Expression::clone
		virtual Expression::Ptr clone() const { return rw::common::ownedPtr( new Real(_value) ); }

		/**
		 * @brief Get the value.
		 * @return the value.
		 */
		double value() const { return _value; }

	private:
		double _value;
	};

	//! @brief A symbol primitive.
	class Symbol: public Expression {
	public:
		//! @brief Smart pointer type
		typedef rw::common::Ptr<Symbol> Ptr;

		/**
		 * @brief Copy constructor.
		 * @param symbol [in] symbol to copy.
		 */
		Symbol(const Symbol& symbol): _name(symbol._name) {}

		/**
		 * @brief Construct new symbol.
		 * @param value [in] the name.
		 */
		Symbol(const char* value): _name(value) {}

		/**
		 * @brief Construct new symbol.
		 * @param name [in] the name.
		 */
		Symbol(const std::string& name): _name(name) {}

		//! @brief Destructor.
		virtual ~Symbol() {}

		//! @copydoc Expression::getType
		Type getType() const { return Expression::Symbol; }

		//! @copydoc Expression::out
		void out(std::ostream& stream) const { stream << _name; }

		//! @copydoc Expression::clone
		virtual Expression::Ptr clone() const { return rw::common::ownedPtr( new Symbol(_name) ); }

		/**
		 * @brief Get the name of the symbol.
		 * @return the name.
		 */
		const std::string getName() const { return _name; }

	private:
		const std::string _name;
	};

	//! @brief A base interface for function expressions.
	class FunctionBase: public Expression {
	public:
		//! @brief Smart pointer type
		typedef rw::common::Ptr<FunctionBase> Ptr;

		/**
		 * @brief Construct new function.
		 * @param name [in] the name of the function.
		 */
		FunctionBase(const std::string& name);

		/**
		 * @brief Get the name of the function.
		 * @return the name of the function.
		 */
		std::string getName() const;

		//! @copydoc Expression::getType
		Type getType() const;

		//! @copydoc Expression::out
		virtual void out(std::ostream& stream) const;

		/**
		 * @brief Get a list of arguments for this function.
		 * @return list of arguments.
		 */
		virtual std::list<rw::common::Ptr<const Expression> > getArguments() const = 0;

		/**
		 * @brief Print function by using indentations.
		 * @param stream [in/out] the output stream.
		 * @param indent [in] the amount of indentation.
		 */
		virtual void out(std::ostream& stream, std::size_t indent) const;

	protected:
		//! @brief Name of the function.
		std::string _name;
	};

	//! @brief A user definable function expression.
	class Function: public FunctionBase {
	public:
		//! @brief Smart pointer type
		typedef rw::common::Ptr<Function> Ptr;

		/**
		 * @brief Construct new function.
		 * @param name [in] the name of the function.
		 */
		Function(const std::string& name);

		/**
		 * @brief Change the name of the function.
		 * @param name [in] the new name.
		 */
		void setName(const std::string &name);

		/**
		 * @brief Append argument.
		 * @param arg [in] the argument.
		 */
		void addArgument(const AutoExpression& arg);

		/**
		 * @brief Append argument.
		 * @param arg [in] the argument.
		 */
		void addArgument(const Expression::Ptr arg);

		/**
		 * @brief Append list of arguments.
		 * @param args [in] the list.
		 */
		void addArguments(const std::list<Expression::Ptr>& args);

		/**
		 * @brief Change argument.
		 * @param i [in] index.
		 * @param arg [in] new argument.
		 */
		void setArgument(std::size_t i, Expression::Ptr arg);

		//! @copydoc FunctionBase::getArguments
		virtual std::list<rw::common::Ptr<const Expression> > getArguments() const;

		//! @copydoc FunctionBase::getArguments
		virtual std::list<Expression::Ptr> getArguments();

		//! @copydoc Expression::clone
		virtual Expression::Ptr clone() const;

	private:
		std::list<Expression::Ptr> _arguments;
	};

	//! @brief An Array primitive.
	template<typename T>
	class Array: public Expression {
	public:
		//! @brief Smart pointer type
		typedef rw::common::Ptr<Array<T> > Ptr;

		//! @brief Destructor.
		virtual ~Array() {}

		//! @copydoc Expression::getType
		Type getType() const { return Expression::Array; }

		/**
		 * @brief Get the shape as a list of integers.
		 * @return list of same length as dimensions()
		 */
		virtual const int* size() const = 0;

		/**
		 * @brief Get the dimensions.
		 * @return the number of dimensions.
		 */
		virtual int dimensions() const = 0;

		/**
		 * @brief Get the raw data.
		 * @return a pointer to the first element.
		 */
		virtual const T* data() const = 0;

	protected:
		Array() {}
	};

	//! @brief Convenience class for automatic Expression deduction.
	class AutoExpression {
	public:
		/**
		 * @brief Construct from other expression.
		 * @param val [in] the value.
		 */
		AutoExpression(const Expression& val): _exp(val.clone()) {}

		/**
		 * @brief Construct from double.
		 * @param val [in] the value.
		 */
		AutoExpression(double val): _exp(rw::common::ownedPtr(new Real(val))) {}

		/**
		 * @brief Construct from float.
		 * @param val [in] the value.
		 */
		AutoExpression(float val): _exp(rw::common::ownedPtr(new Real(val))) {}

		/**
		 * @brief Construct from int.
		 * @param val [in] the value.
		 */
		AutoExpression(int val): _exp(rw::common::ownedPtr(new Integer(val))) {}

		/**
		 * @brief Construct from bool.
		 * @param val [in] the value.
		 */
		AutoExpression(bool val): _exp(rw::common::ownedPtr(new Symbol(val?"True":"False"))) {}

		/**
		 * @brief Construct from string.
		 * @param string [in] the value.
		 */
		AutoExpression(const std::string& string): _exp(rw::common::ownedPtr(new String(string))) {}

		/**
		 * @brief Construct from string.
		 * @param string [in] the value.
		 */
		AutoExpression(const char* string): _exp(rw::common::ownedPtr(new String(string))) {}

		/**
		 * @brief Construct from list of arguments.
		 * @param args [in] the arguments.
		 */
		AutoExpression(const std::list<AutoExpression>& args);

#if __cplusplus >= 201103L
		/**
		 * @brief Construct from list of arguments.
		 * @param args [in] the arguments.
		 * @note Only defined for C++11
		 */
		AutoExpression(const std::initializer_list<Mathematica::AutoExpression>& args);
#endif

		/**
		 * @brief Get expression.
		 * @return the expression.
		 */
		Expression::Ptr expression() const {
			return _exp;
		}

	private:
		const Expression::Ptr _exp;
	};

	//! @brief A Packet expression.
	class Packet: public FunctionBase {
	public:
		//! @brief Smart pointer type.
		typedef rw::common::Ptr<Packet> Ptr;

		/**
		 * @brief Construct new expression.
		 * @param name [in] name of the packet.
		 * @param type [in] the type of the packet.
		 */
		Packet(const std::string& name, PacketType type): FunctionBase(name), _type(type) {}

		//! @brief Destructor.
		virtual ~Packet() {}

		/**
		 * @brief Get the type of packet.
		 * @return the type.
		 */
		PacketType packetType() const { return _type; }

	private:
		const PacketType _type;
	};

private:
	static void put(rw::common::Ptr<const LinkImpl> link, const Expression& expression);
	static void error(rw::common::Ptr<const LinkImpl> link);
	static std::string readString(rw::common::Ptr<const LinkImpl> link, bool symbol);
	static Expression::Ptr readExpression(rw::common::Ptr<const LinkImpl> link);
	static void addExpression(Function::Ptr exp, rw::common::Ptr<const LinkImpl> link);
	static void readRawByteArray(rw::common::Ptr<const LinkImpl> link, std::ostream& stream);
	static int expectFunction(rw::common::Ptr<const LinkImpl> link, const std::string& name);

	struct Environment;

private:
	Environment* _env;
	std::list<rw::common::Ptr<Link> > _links;
};

/**
 * @brief Print a Mathematica expression to an output stream.
 * @param out [in/out] the stream to print to.
 * @param expression [in] the expression to print.
 * @return a reference to the stream for chaining.
 */
std::ostream& operator<<(std::ostream& out, const Mathematica::Expression& expression);

//! @}
} /* namespace mathematica */
} /* namespace rwlibs */
#endif /* RWLIBS_MATHEMATICA_MATHEMATICA_HPP_ */
