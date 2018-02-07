import sst
import sst.macro

opticalLatency = "0ps"
smallLatency = "1ms"

def readSquareMatrix(filename, directory):
	file = open(filename, 'r')
	if file == None:
		return None
	line = file.readln()
	size = int(line)
	i = 0
	matrix = [0] * size
	while line != None:
		line = file.readln()
		entries = line.split(" ")
		row = []
		for element in entries:
			row.append(int(element))
		matrix[i] = row
		i += 1
	#assert(i == size)
	return matrix

def makeUniLink(linkType,srcComp,srcId,srcPort,dstComp,dstId,dstPort,outLat=None,inLat=None):
	if not outLat : outLat = inLat
	if not inLat: inLat = outLat
	if not outLat: sys.exit("must specify at least one latency for link")

	linkName = "%s%d:%d->%d:%d" % (linkType,srcId,srcPort,dstId,dstPort)
	link = sst.Link(linkName)
	portName = "output %d %d" % (srcPort, dstPort)
	srcComp.addLink(link,portName,outLat)
	portName = "input %d %d" % (srcPort, dstPort)
	dstComp.addLink(link,portName,inLat)

def makeUniNetworkLink(srcComp,srcId,srcPort,dstComp,dstId,dstPort,outLat=None,inLat=None):
	makeUniLink("network",srcComp,srcId,srcPort,dstComp,dstId,dstPort,outLat,inLat)

def makeBiLink(linkType,comp1,id1,port1,comp2,id2,port2,outLat=None,inLat=None):
	makeUniLink(linkType,comp1,id1,port1,comp2,id2,port2,outLat,inLat)
	makeUniLink(linkType,comp2,id2,port2,comp1,id1,port1,outLat,inLat)

class DragonflyTopology:
	def __init__(self, aArg, gArg, hArg, needRouterArg):
		self.a = aArg
		self.g = gArg
		self.h = hArg
		self.matrix = None
		self.needRouter = needRouterArg
		self.totalSwitches = self.a * self.g
		self.switches = [0] * self.totalSwitches
		switchName = "dragonfly_switch"
		for i in range(self.totalSwitches):
			currSwitch = sst.Component("Switch %d" % optical_switch_id, "macro.%s" % switchName)
			currSwitch.addParam("id", i)
			self.switches[i] = currSwitch
		self.setup()
		return
	
	# Have to assume that the topology is symmetrical, that means that all links
	# are bidirectional between switch pairs, and that also means that the adjacency matrix will
	# have to be symmetrical about the diagonal
	# NOTE: This forms the actual topology with each link in the simualation being the actual link in the
	# 		HPC network. The downside to this way of forming the topology is that we have to figure out how 
	# 		to actually route the simulation
	def formPhysicalTopology(self):
		inports = [0] * self.totalSwitches
		outports = [0] * self.totalSwitches
		for i in range(self.totalSwitches):
			currID = i
			currSwitch = self.switches[currID]
			numports = 0
			for j in range(i + 1, self.totalSwitches, 1):
				if self.matrix[i][j] == 0:
					continue	
				assert(self.matrix[j][i] == self.matrix[i][j])
				targetID = j
				targetSwitch = self.switches[targetID]
				makeUniNetworkLink(currSwitch, currID, outports[currID], targetSwitch, targetID, inports[targetID], smallLatency)
				makeUniNetworkLink(targetSwitch, targetID, outports[targetID], currSwitch, currID, inports[currID], smallLatency)
				numports += 1
				currSwitch.addParam("outport:%d" % outports[currID], targetID)
				targetSwitch.addParam("inport:%d" % inports[targetID], currID)
				currSwitch.addParam("inport:%d" % inports[currID], targetID)
				targetSwitch.addParam("outport:%d" % outports[targetID], currID)
				inports[currID] += 1
				inports[targetID] += 1
				outports[currID] += 1
				outports[targetID] += 1
			currSwitch.addParam("numports", numports)
		return

	def formVirtualTopology(self):
		inports = [0] * self.totalSwitches
		outports = [0] * self.totalSwitches
		for i in range(self.totalSwitches):
			for j in range(i + 1, self.totalSwitches, 1):
				assert(self.matrix[j][i] == self.matrix[i][j])
				targetID = j
				targetSwitch = self.switches[targetID]
				makeUniNetworkLink(currSwitch, currID, outports[currID], targetSwitch, targetID, inports[targetID], smallLatency)
				makeUniNetworkLink(targetSwitch, targetID, outports[targetID], currSwitch, currID, inports[currID], smallLatency)
				numports += 1
				currSwitch.addParam("outport:%d" % outports[currID], targetID)
				targetSwitch.addParam("inport:%d" % inports[targetID], currID)
				currSwitch.addParam("inport:%d" % inports[currID], targetID)
				targetSwitch.addParam("outport:%d" % outports[targetID], currID)
				inports[currID] += 1
				inports[targetID] += 1
				outports[currID] += 1
				outports[targetID] += 1
			currSwitch.addParam("numports", numports)
		return

	def setup(self):
		filename = "adjacency_a-%d_g-%d_h-%d" % (self.a, self.g, self.h)
		readMatrix = readSquareMatrix(filename)
		self.matrix = readMatrix
		if self.needRouter:
			self.formPhysicalTopology()
		else:
			self.formVirtualTopology()
		return


class Interconnect:
	def __init__(self, params):
		self.params = params
		self.system = sst.macro.System(params)
		self.num_nodes = self.system.numNodes()
		self.num_switches = self.system.numSwitches()

		self.switches = [0]*self.num_switches
		self.nodes = [0]*self.num_nodes #just expands the size of an initial array with 1 element 0 to size 1 * num_nodes
		topolParams = self.params["topology"]
		self.switches_per_group = int(topolParams["switches_per_group"])
		self.nodes_per_switch = int(topolParams["nodes_per_switch"])
		self.num_groups = int(topolParams["groups"])
		assert(self.num_switches == self.num_groups * self.switches_per_group)
		self.switchRadices = [0] * self.num_switches
					
	def buildEndpoints(self):
		nodeParams = self.params["node"]
		modelName = nodeParams["model"]
		for i in range(self.num_nodes): 
			self.nodes[i] = sst.Component("Node %d" % i, "macro.%s" % (modelName + "_node"))
			self.nodes[i].addParams(macroToCoreParams(nodeParams))
			self.nodes[i].addParam("id", i)       
		return

	def buildElectricalSwitches(self):
		switchParams = self.params["switch"]
		switchName = "dragonfly" + "_switch"
		#totalSwitchesPerGroup = self.optical_switches_per_group + self.switches_per_group
		for i in range(self.num_switches):
			#if self.containsOptics and i % (totalSwitchesPerGroup) >= self.switches_per_group:
				#continue
			switch = sst.Component("Switch %d" % i, "macro.%s" % switchName)
			switch.addParam("id" , i) 
			switch.addParams(macroToCoreParams(switchParams))
			switch.addParam("switches_per_group", self.switches_per_group)
			switch.addParam("nodes_per_switch", self.nodes_per_switch)
			self.switches[i] = switch 
		return

	def latency(self, params):
		if params.has_key("latency"):
			return params["latency"]
		elif params.has_key("send_latency"):
			return params["send_latency"]
		else:
			sys.exit("need link latency in parameters")

	def buildTopology(self):
		switchParams = self.params["switch"]
		for i in range(self.num_switches):
			linkParams = switchParams["link"]
			connections = self.system.switchConnections(i)
			srcSwitch = self.switches[i]
			lat = self.latency(linkParams)
			for srcId, dstId, srcOutport, dstInport in connections:
				#print "srcId: %d and dstId: %d srcOutport: %d, dstInport: %d" % (srcId, dstId, srcOutport, dstInport)
				#print self.switches
				#print "connecting - srcID: %d outport: %d -> dstID: %d inport: %d" % (srcId,	 srcOutport, dstId, dstInport)
				srcSwitch.addParam("switch_radix", len(connections))
				dstSwitch = self.switches[dstId]
				self.switchRadices[i] = len(connections)
				#linkName = "network %d:%d->%d:%d" % (srcId, srcOutport ,dstId, dstInport)
        		#link = sst.Link(linkName)
        		#portName = "output %d %d" % (srcOutport, dstInport)
        		#srcSwitch.addLink(link, portName, lat)
        		#portName = "input %d %d" % (srcOutport, dstInport)
        		#dstSwitch.addLink(link, portName, lat)
				makeUniNetworkLink(srcSwitch, srcId, srcOutport,
									dstSwitch, dstId, dstInport, lat)
		return

	def buildNodeConnections(self):
		for i in range(self.num_groups * self.switches_per_group):
			switch = self.switches[i]
			for nodeIndex in range(self.nodes_per_switch):
				index = nodeIndex + (i * self.nodes_per_switch)
				node = self.nodes[index]
				switchPortIndex = self.switchRadices[i] + nodeIndex
				makeUniLink("injection",node,index,0,switch,i,switchPortIndex,smallLatency)
				makeUniLink("ejection",switch,i,switchPortIndex,node,index,0,smallLatency)

	def buildNodeConnections2(self):
		for i in range(self.num_groups * self.switches_per_group):
			switch = self.switches[i]
			for nodeIndex in range(self.nodes_per_switch):
				index = nodeIndex + (i * self.nodes_per_switch)
				node = self.nodes[index]
				switchPortIndex = self.switches_per_group + nodeIndex
				linkName = "logP %d -> %d" % (index, i)
				link = sst.Link(linkName)
				print "NICLogInjectionPort: %d and SwitchLogPInjectionPort: %d" % (sst.macro.NICLogPInjectionPort, sst.macro.SwitchLogPInjectionPort)
				portName = "in-out %d %d" % (sst.macro.NICLogPInjectionPort, sst.macro.SwitchLogPInjectionPort)
				node.addLink(link, portName, smallLatency)
				portName = "in-out %d %d" % (i, sst.macro.SwitchLogPInjectionPort)
				switch.addLink(link, portName, smallLatency)


	def buildLogPNetwork(self):
		import re
		nproc = sst.getMPIRankCount() * sst.getThreadCount()
		switchParams = self.params["switch"]
		linkParams = switchParams["link"]
		ejParams = switchParams["ejection"]
		lat = self.latency(ejParams)
		#gotta multiply the lat by 2
		match = re.compile("(\d+[.]?\d*)(.*)").search(lat)
		if not match:
			sys.exit("improperly formatted latency %s" % lat)
		num, units = match.groups()
		num = eval(num) * 2
		lat = "%8.4f%s" % (num,units.strip())
		switches = []
		for i in range(nproc):
			switch = sst.Component("logp %d" % i, "macro.logp_switch")
			#switch = sst.Component("logp %d" % i, "macro.logp_switch")
			switch.addParams(macroToCoreParams(switchParams))
			switch.addParam("id", i)
			switches.append(switch)
			#NEWLY ADDED BEGIN
			#switch.addParam("electrical_bandwidth" , "10Gb/s")
			#switch.addParam("optical_bandwidth", "1Gb/s")
			#NEWLY ADDED END
		for i in range(nproc):
			sw_i = switches[i]
			for j in range(nproc):
				sw_j = switches[j]
				if i==j: continue
				linkName = "logPnetwork%d->%d" % (i,j)
				link = sst.Link(linkName)
				portName = "in-out %d %d" % (j, sst.macro.SwitchLogPNetworkPort)
				sw_i.addLink(link, portName, lat)
				portName = "in-out %d %d" % (i, sst.macro.SwitchLogPNetworkPort)
				sw_j.addLink(link, portName, lat)			
		
		for i in range(self.num_nodes):
			injSW = self.system.nodeToLogPSwitch(i)
			ep = self.nodes[i]
			sw = switches[injSW]
			linkName = "logPinjection%d->%d" % (i, injSW)
			link = sst.Link(linkName)
			portName = "in-out %d %d" % (sst.macro.NICLogPInjectionPort, sst.macro.SwitchLogPInjectionPort)
			#portName = "injection %d %d" % (sst.macro.NICLogPInjectionPort, sst.macro.SwitchLogPInjectionPort)
			ep.addLink(link, portName, smallLatency) #put no latency here
			portName = "in-out %d %d" % (i, sst.macro.SwitchLogPInjectionPort)
			#portName = "ejection %d %d" % (i, sst.macro.SwitchLogPInjectionPort)
			sw.addLink(link, portName, smallLatency)


	def build(self, islogP):
		if not islogP:
			
			self.buildEndpoints()

			self.buildElectricalSwitches()
			self.buildTopology()
			self.buildNodeConnections()
			self.buildLogPNetwork()
		else:
			self.buildEndpoints()
			self.buildSimplifiedTopology(smallLatency)
			self.buildLogPNetwork()
		

## Returns the command line argv in terms of a vector that is 0-indexed
def readCmdLineParams():
	import sys
	return sst.macro.readParams(sys.argv)

def redoSubParams_impl(nsArr, theDict, allParams):
	for key in theDict:
		val = theDict[key]
		if isinstance(val, dict):
			newNsArr = nsArr[:]
			newNsArr.append(key)
			redoSubParams_impl(newNsArr, val, allParams)
		else:
			paramArr = nsArr[:]
			paramArr.append(key)
			newParam = ".".join(paramArr)
			allParams.append((newParam, val))

def macroToCoreParams(theDict):
	allParams = []
	redoSubParams_impl([], theDict, allParams)
	newDict = {}
	for key, val in allParams:
		newDict[key] = val
	return newDict

def setupTopology():
	import sys
	params = readCmdLineParams()
	nodeParams = params["node"]
	swParams = params["switch"]
	builtinApps = [
   		"apitest",
       	"global_test",
       	"hello_world",
       	"mpi_coverage",
       	"mpi_ping_all",
       	"mpi_print_nodes",
       	"mpi_topology",
       	"parsedumpi",
       	"sstmac_mpi_testall",
       	"traffic_matrix",
       	"user_app_cxx_empty_main",
       	"user_app_cxx_full_main",
	]

	for appIdx in range(10):
		appKey = "app%d" % (appIdx)
		#nodeParams = params["node"]
		if params.has_key(appKey):
			appParams = params[appKey]
			nodeParams[appKey] = appParams
			appName = appParams["name"]
			if not appName in builtinApps:
				cmd = "import sst.%s" % appName
				exec(cmd)
			del params[appKey]

	debugList = []
	if params.has_key("debug"):
		debugList = params["debug"].strip().split()
	for i in range(len(sys.argv)):
		if sys.argv[i] == "-d" or sys.argv[i] == "--debug":
			debugList.extend(sys.argv[i+1].split(","))

	icParams = {}
	icParams["topology"] = params["topology"]
	nodeParams["interconnect"] = icParams
	if debugList:
		nodeParams["debug"] = " ".join(debugList)
	swParams["topology"] = params["topology"]

	#swParams["topology"] = "torus"
	#move every param in the global namespace 
	#into the individal namespaces
	for ns in "node", "switch":
		nsParams = params[ns]
		for key in params:
			val = params[key]
			if isinstance(val, str):
				if not nsParams.has_key(key):
					nsParams[key] = val    
	interconnect = Interconnect(params)
	isLogP = False
	interconnect.build(isLogP)
