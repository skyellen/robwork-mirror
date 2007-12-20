#include "PGMLoader.hpp"

#include <boost/spirit.hpp>
#include <boost/spirit/core.hpp>
#include <boost/spirit/actor.hpp>
#include <boost/spirit/dynamic.hpp>
#include <boost/spirit/symbols.hpp>
#include <boost/spirit/phoenix.hpp>
#include <boost/spirit/error_handling.hpp>
#include <boost/spirit/iterator.hpp>

#include <rw/common/StringUtil.hpp>
#include <rw/common/IOUtil.hpp>
#include <rw/common/macros.hpp>

using namespace rw;
using namespace rw::common;
using namespace rw::loaders;
using namespace rw::sensor;

using namespace boost::spirit;
using namespace phoenix;


namespace {

	struct DataParser
    {
        typedef char result_t;
        template <typename ScannerT>
        std::ptrdiff_t
        operator()(ScannerT const& scan, result_t& result) const
        {
            if (scan.at_end())
                return -1;

            result = *scan;
            std::ptrdiff_t len = 1;
            ++scan;
            return len;
        }
    };

    functor_parser<DataParser> DataParser_p;

	struct ResizeVector {
	    ResizeVector(std::vector<char>& v, int &width, int &height, int factor):
	    	_v(v),_width(width),_height(height),_factor(factor)
	        {}
	
        template < typename IteratorT >
        void operator()(IteratorT const& first, IteratorT const& last) const {
        	std::cout << "Resizeing to: " << _width*_height*_factor << " factor: " << _factor << std::endl;
	    	_v.resize(_width*_height*_factor);
	    }
	    
	    std::vector<char> &_v;
	    int &_width,&_height,_factor;
	};
	
	struct InsertAtVector {
		InsertAtVector(std::vector<char>& v, int &index):
	    	_v(v),_index(index)
	        {}
	
        
        void operator()(char value) const {
	    	_v[_index] = value;
	    }
	    
	    std::vector<char> &_v;
	    int &_index;
	};
	
	

	struct PGMParser: grammar<PGMParser>
	{
	public:		
	    mutable std::vector<char>& _data;
	    mutable int width, height, maxgrayval;
	    
	    
	    PGMParser(std::vector<char> &data):_data(data){}
	    
	    template <typename ScannerT>
	    struct definition {
	    public:
	    	int index;
	    	std::vector<char> &data;
	    	int &width, &height, &maxgrayval;
	    	
	    	
	        definition(PGMParser const &self): data(self._data),
	        	width(self.width),height(self.height),maxgrayval(self.maxgrayval)
	        {
	        	
	        	pgmfile_r = header_r >>
	        		dimension_r >> 
	        		maxgrayval_r >>
	        		data_r >>
	        		end_p;
	        	
	        	// expect magic number identifier
	        	header_r = str_p("P5")>>
	        			   *(ch_p('#') >> lexeme_d[*(anychar_p-eol_p)]);
	        	
	        	dimension_r = int_p[var(width) = arg1] //[var(std::cout) << "Width:" << arg1 << std::endl]  
	        				  >> int_p[var(height) = arg1]//[var(std::cout) << "height:" << arg1 << std::endl]
	        				 ;
	
	        	maxgrayval_r = int_p[var(maxgrayval)=arg1];
	        	
	        	data_r = if_p( var(maxgrayval)<256 )[data8bit_r].else_p[data16bit_r];
	        		
	        	data8bit_r = eps_p[ ResizeVector(data,width,height,1) ] >> 
	        		lexeme_d[
	        		for_p(var(index) = 0, var( index ) < var(width)*var(height), var(index) += 1 )
	        			[ DataParser_p[ InsertAtVector( data, index ) ]
	        			//[ anychar_p[ assign_key_a( var(data), var(index)) ]//[std::cout << var(index)<< " " << var(width)*var(height) << std::endl] 
	        			  | eps_p[ var( std::cout ) << "WARNING: PGM file is not well formed, missing data! Continues parsing" << std::endl ]
	        			]
	        		];
	        	
	        	data16bit_r = eps_p[ ResizeVector(data,height,width,2) ] >> 
	        		lexeme_d[
	        		for_p(var(index) = 0, var( index ) < var(width)*var(height)*2, var(index) += 1 )
	        			[ anychar_p[ assign_key_a( var(data), var(index)) ]//[std::cout << var(index)<< std::endl] 
	        			 | eps_p[ var( std::cout ) << "WARNING: PGM file is not well formed, missing data! Continues parsing" << std::endl ]                                                    
	        			]
	        		];       		
	        	
	        }
	
	        boost::spirit::rule<ScannerT> const start() const 
	        { 
	            return pgmfile_r; 
	        }
	
	    private:
	        boost::spirit::rule<ScannerT > 
	            pgmfile_r, header_r, dimension_r, maxgrayval_r, data_r, data8bit_r, data16bit_r;
	    };
	};    
}

std::auto_ptr<rw::sensor::Image> PGMLoader::Load(const std::string& filename){
	
    std::vector<char> input;
    IOUtil::ReadFile(filename, input);
    std::cout << "Size of input: " << input.size() << std::endl;
    
    typedef position_iterator<std::vector<char>::const_iterator > iterator_t;
    iterator_t first(input.begin(),input.end());
    iterator_t last;
    std::vector<char> *output = new std::vector<char>();
    PGMParser p(*output);


    /* TODO: should append to output instead of assigning */
    boost::spirit::parse_info<iterator_t> info =
        boost::spirit::parse( first, last, p, space_p[var(std::cout) << "SKIP" << arg1 << std::endl ]);
    
    if( !info.hit ){
    	delete output;
        RW_THROW("Error parsing file: "<< filename);
    }
    Image::ColorCode coding = Image::MONO16;
    if(p.maxgrayval<256)
    	coding = Image::MONO8;
    	
    std::auto_ptr<Image> result(new Image(*output,p.width,p.height,coding));
    std::cout << "size of output: " << output->size() << std::endl;
    return result;
}
