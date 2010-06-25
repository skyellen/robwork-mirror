#ifndef CONTACTPOINTFILTER_HPP_
#define CONTACTPOINTFILTER_HPP_


class ContactPointFilter {
	
public:
	
	/*ContactPointFilter(){
		
	}*/
	
	static void Filter(std::vector<Vector3D<> >& points, 
				std::vector<double>& dist,
				std::list<std::pair<Vector3D<>,int > >& blobs,
				double sepDist)
	{
		//std::vector<int> blobs;
		for(int i=0; i<points.size(); i++){
			const Vector3D<> &p = points[i]; 
			bool inBlob = false; 
			for(int j=0; j<blobs.size(); j++){
				double dist = MetricUtil::Norm2(p,blobs[j].first);
				if( dist < sepDist ){
					// point is inside blob, break loop
					blobs[j].second++;
					inBlob = true;
					break;
				} else if(dist<sepDist*2) {
					// discard point
					inBlob = true;
					break;
				}
			}
			if( !inBlob )
				blobs.push_back(i);
		}
		
	}
	
};



#endif /*CONTACTPOINTFILTER_HPP_*/
