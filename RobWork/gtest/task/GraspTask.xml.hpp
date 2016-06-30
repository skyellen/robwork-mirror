/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef ROBWORK_GTEST_TASK_GRASPTASK_XML_HPP_
#define ROBWORK_GTEST_TASK_GRASPTASK_XML_HPP_

#include <string>

static const std::string& getGraspTaskXML() {
	// Generate with: cat grasptask.xml | sed 's/\"/\\\"/g' | awk '{ print $0, "\\" }'
	static const std::string xml = "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?> \
	<CartesianTask> \
	 \
	  <Targets/> \
	 \
	  <Entities> \
	    <CartesianTask> \
	      <Targets> \
	        <CartesianTarget id=\"0\"> \
	          <Transform3D> \
	            <Pos>-0.2459544 -0.8678307 0.4317132</Pos> \
	            <Rotation3D>0.2549632 -0.8745079 0.4125889 -0.5708913 0.2082455 0.7941769 -0.7804338 -0.4380293 -0.446154</Rotation3D> \
	          </Transform3D> \
	          <Id></Id> \
	          <Index>0</Index> \
	          <PropertyMap> \
	            <Property> \
	              <Name>ContactsGrasp</Name> \
	              <Value> \
	                <DoubleList>0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0</DoubleList> \
	              </Value> \
	            </Property> \
	            <Property> \
	              <Name>ContactsLift</Name> \
	              <Value> \
	                <DoubleList>0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0</DoubleList> \
	              </Value> \
	            </Property> \
	            <Property> \
	              <Name>GripperConfiguration</Name> \
	              <Value> \
	                <Q>-0.03584265 0.1980277 -0.9373505 -0.2843841</Q> \
	              </Value> \
	            </Property> \
	            <Property> \
	              <Name>GripperConfigurationPost</Name> \
	              <Value> \
	                <Q>0.7606183 -0.2080465 0.6000562 -0.1345698</Q> \
	              </Value> \
	            </Property> \
	            <Property> \
	              <Name>Interference</Name> \
	              <Value> \
	                <Double>1.123</Double> \
	              </Value> \
	            </Property> \
	            <Property> \
	              <Name>InterferenceAngles</Name> \
	              <Value> \
	                <DoubleList>0.4 0.5 0.6</DoubleList> \
	              </Value> \
	            </Property> \
	            <Property> \
	              <Name>InterferenceDistances</Name> \
	              <Value> \
	                <DoubleList>0.1 0.2 0.3 0.4</DoubleList> \
	              </Value> \
	            </Property> \
	            <Property> \
	              <Name>Interferences</Name> \
	              <Value> \
	                <DoubleList>0.7 0.8</DoubleList> \
	              </Value> \
	            </Property> \
	            <Property> \
	              <Name>LiftResult</Name> \
	              <Value> \
	                <Double>0.123</Double> \
	              </Value> \
	            </Property> \
	            <Property> \
	              <Name>LiftStatus</Name> \
	              <Value> \
	                <Integer>1</Integer> \
	              </Value> \
	            </Property> \
	            <Property> \
	              <Name>ObjectTtcpApproach</Name> \
	              <Value> \
	                <Transform3D> \
	                  <Pos>0.25201 -0.0695093 -0.965225</Pos> \
	                  <Rotation3D>-0.00270293 -0.89361 0.448835 0.67105 -0.334393 -0.66172 0.741407 0.299402 0.600561</Rotation3D> \
	                </Transform3D> \
	              </Value> \
	            </Property> \
	            <Property> \
	              <Name>ObjectTtcpGrasp</Name> \
	              <Value> \
	                <Transform3D> \
	                  <Pos>0.154929 0.987479 0.0296881</Pos> \
	                  <Rotation3D>-0.455489 -0.884303 -0.102657 -0.663233 0.413999 -0.623479 0.593844 -0.215902 -0.775071</Rotation3D> \
	                </Transform3D> \
	              </Value> \
	            </Property> \
	            <Property> \
	              <Name>ObjectTtcpLift</Name> \
	              <Value> \
	                <Transform3D> \
	                  <Pos>0.2288791 -0.8427268999999999 0.4872634</Pos> \
	                  <Rotation3D>0.8744951 -0.4795291 0.07287096 0.08946266 0.3071274 0.947454 -0.4767124 -0.8220247000000001 0.3114814</Rotation3D> \
	                </Transform3D> \
	              </Value> \
	            </Property> \
	            <Property> \
	              <Name>ObjectTtcptTarget</Name> \
	              <Value> \
	                <Transform3D> \
	                  <Pos>-0.841704 0.528854 -0.108845</Pos> \
	                  <Rotation3D>0.449831 0.07166409999999999 0.890234 -0.269712 0.961137 0.0589122 -0.851415 -0.266607 0.451678</Rotation3D> \
	                </Transform3D> \
	              </Value> \
	            </Property> \
	            <Property> \
	              <Name>QualityAfterLifting</Name> \
	              <Value> \
	                <Q>0.2510191 -0.06923598 -0.9614297000000001 0.08859286</Q> \
	              </Value> \
	            </Property> \
	            <Property> \
	              <Name>QualityBeforeLifting</Name> \
	              <Value> \
	                <Q>0.2719389 0.4146291 0.853626 -0.1595451</Q> \
	              </Value> \
	            </Property> \
	            <Property> \
	              <Name>TestStatus</Name> \
	              <Value> \
	                <Integer>1</Integer> \
	              </Value> \
	            </Property> \
	          </PropertyMap> \
	        </CartesianTarget> \
	        <CartesianTarget id=\"1\"> \
	          <Transform3D> \
	            <Pos>0.3580329 0.5530171 0.7523194</Pos> \
	            <Rotation3D>-0.6115657 0.7761471 0.1535677 -0.7856076 -0.6187233 -0.001500034 0.09385167 -0.1215613 0.988137</Rotation3D> \
	          </Transform3D> \
	          <Id></Id> \
	          <Index>1</Index> \
	          <PropertyMap/> \
	        </CartesianTarget> \
	        <CartesianTarget id=\"2\"> \
	          <Transform3D> \
	            <Pos>-0.202799 -0.5613984 -0.8023119</Pos> \
	            <Rotation3D>0.8131517 -0.4044635 -0.4185614 0.3927172 0.9120122000000001 -0.1183505 0.4296015 -0.06813937 0.9004441</Rotation3D> \
	          </Transform3D> \
	          <Id></Id> \
	          <Index>2</Index> \
	          <PropertyMap/> \
	        </CartesianTarget> \
	      </Targets> \
	      <Entities/> \
	      <Id>taskId1</Id> \
	      <Index>0</Index> \
	      <PropertyMap> \
	        <Property> \
	          <Name>Approach</Name> \
	          <Value> \
	            <Transform3D> \
	              <Pos>-0.599348 0.144885 0.787268</Pos> \
	              <Rotation3D>0.021466 0.478732 -0.877699 0.823371 0.489505 0.287133 0.567098 -0.728835 -0.383666</Rotation3D> \
	            </Transform3D> \
	          </Value> \
	        </Property> \
	        <Property> \
	          <Name>CloseQ</Name> \
	          <Value> \
	            <Q>0.6743709 0.6914217 0.2217055 -0.1341887</Q> \
	          </Value> \
	        </Property> \
	        <Property> \
	          <Name>ObjectID</Name> \
	          <Value> \
	            <String>objectId1</String> \
	          </Value> \
	        </Property> \
	        <Property> \
	          <Name>Offset</Name> \
	          <Value> \
	            <Transform3D> \
	              <Pos>0.128173 -0.297606 0.9460460000000001</Pos> \
	              <Rotation3D>-0.74816 -0.220389 0.625848 -0.445398 0.865945 -0.227507 -0.49181 -0.448963 -0.746026</Rotation3D> \
	            </Transform3D> \
	          </Value> \
	        </Property> \
	        <Property> \
	          <Name>OpenQ</Name> \
	          <Value> \
	            <Q>0.1081486 -0.2511118 0.7982471 -0.5367013</Q> \
	          </Value> \
	        </Property> \
	        <Property> \
	          <Name>Retract</Name> \
	          <Value> \
	            <Transform3D> \
	              <Pos>-0.0373863 0.206556 -0.97772</Pos> \
	              <Rotation3D>0.8233549999999999 0.407011 0.395511 -0.410789 0.908257 -0.0795062 -0.391585 -0.0970095 0.915014</Rotation3D> \
	            </Transform3D> \
	          </Value> \
	        </Property> \
	        <Property> \
	          <Name>TauMax</Name> \
	          <Value> \
	            <Q>-0.3795243 -0.4163515 0.6758378 0.4752431</Q> \
	          </Value> \
	        </Property> \
	        <Property> \
	          <Name>refframe</Name> \
	          <Value> \
	            <String>refframe1</String> \
	          </Value> \
	        </Property> \
	      </PropertyMap> \
	    </CartesianTask> \
	  </Entities> \
	 \
	  <Id></Id> \
	 \
	  <Index>-1</Index> \
	 \
	  <PropertyMap> \
	    <Property> \
	      <Name>GraspController</Name> \
	      <Value> \
	        <String>GraspControllerId</String> \
	      </Value> \
	    </Property> \
	    <Property> \
	      <Name>Gripper</Name> \
	      <Value> \
	        <String>GripperId</String> \
	      </Value> \
	    </Property> \
	    <Property> \
	      <Name>TCP</Name> \
	      <Value> \
	        <String>TCPId</String> \
	      </Value> \
	    </Property> \
	  </PropertyMap> \
	 \
	</CartesianTask> \
	";
	return xml;
}

#endif /* ROBWORK_GTEST_TASK_GRASPTASK_XML_HPP_ */
