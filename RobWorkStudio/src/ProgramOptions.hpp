#ifndef RWS_PROGRAMOPTIONS_HPP_
#define RWS_PROGRAMOPTIONS_HPP_

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>

#include <rw/math/Q.hpp>
#include <boost/foreach.hpp>



template<class TYPE>
struct Option {
public:
	Option(std::string n, TYPE val): name(n),value(val){}
	std::string name;
	TYPE value;
};

/**
 * for each of the possible option types a validator is needed.
 */
typedef std::vector< Option<std::string> > StringOptionList;
typedef std::vector< Option<int> > IntOptionList;
typedef std::vector< Option<double> > DoubleOptionList;
typedef std::vector< Option<rw::math::Q> > QOptionList;

//////////////// VALIDATORS
/**
 * @brief validator for properties of type string
 */
void validate(boost::any& v,
		const std::vector<std::string>& values,
		StringOptionList* target_type,
		int);

/**
 * @brief validator for properties of type int
 */
void validate(boost::any& v,
		const std::vector<std::string>& values,
		IntOptionList* target_type,
		int);

/**
 * @brief validator for properties of type double
 */
void validate(boost::any& v,
		const std::vector<std::string>& values,
		DoubleOptionList* target_type,
		int);

/**
 * @brief validator for properties of type Q
 */
void validate(boost::any& v,
		const std::vector<std::string>& values,
		QOptionList* target_type,
		int);

/**
 * @brief handles the validation error/success
 */
template<class TYPE>
void handleResult(bool success, boost::any& v, const std::string& name, const TYPE& value){
	if ( success ) {
		try {
			std::vector<Option<TYPE> >& values =
				boost::any_cast< std::vector<Option<TYPE> >& >(v);
			values.push_back(Option<TYPE>(name,value));
		} catch (...){
			std::vector<Option<TYPE> > values;
			values.push_back(Option<TYPE>(name,value));
			v = boost::any( values );
		}
	} else {
		//throw boost::program_options::validation_error(
		//		boost::program_options::validation_error::invalid_option_value,
		//		"invalid property value, use syntax: <name>=<value>");
	    RW_THROW("invalid property value, use syntax: <name>=<value>");
	}
}

#endif
