
#!/usr/bin/env python
import numpy
import rospy
import yaml
import copy
import os
import sys
import optparse
import matplotlib.pyplot as plt
import collections


class presenter:

    def __init__(self):
        self.filepath = "~"
        self.yaml_file = {}
        self.testblock = []
        self.metric = set()
        self.data = {}
        self.tests = {}
        self.testnames = []
        self.testlist = {}
        self.pos = 0
        self.numvals = {}
        self.min_trans = ['', sys.float_info.max]
        self.min_rot = ['', sys.float_info.max]
        self.min_med_trans = ['', sys.float_info.max]
        self.min_med_rot = ['', sys.float_info.max]



    def import_yaml(self, file):
        with open(file, 'r') as stream:
            print "import file", file
            try:
                #print(yaml.load(stream))
                self.yaml_file = yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        for testblock, metrics in self.yaml_file.iteritems():
            for metric, values in metrics.iteritems():
                self.data.update({metric : []})


    def extract_yaml(self, num):
        avg = copy.deepcopy(self.data)
        mini = copy.deepcopy(self.data)
        maxi = copy.deepcopy(self.data)
        self.numvals[str(num)] = {}
        self.numvals[str(num)]["trans"] = []
        self.numvals[str(num)]["rot"] = []
        print "len: " + str(len(self.numvals))
        for testblock, metrics in self.yaml_file.iteritems():
            #print ("testblock: ", testblock)
            for metric, data in metrics.iteritems():
                #print ("metric: ", metric)
                self.metric.add(metric)
                for values in data:
                    for name, result in values.iteritems():
                        if name == 'data':
                            i = 0
                            j = 0
                            for data_name in result:
                                if isinstance(result[data_name], collections.Iterable):
                                    for data_point in result[data_name]:
                                        if data_name == "trans":
                                            #print str(i) +": "+ str(data_point)
                                            i = i + 1
                                            self.numvals[str(num)]["trans"].append(data_point)
                                        elif data_name == "rot":
                                            #print str(j) + ": " + str(data_point)
                                            j = j + 1
                                            self.numvals[str(num)]["rot"].append(data_point)
                                else:
                                    self.numvals[str(num)][data_name] = result[data_name]


                            #average = sum(self.numvals["trans"])/len(self.numvals["trans"])
                            #print "avg_trans: "+ str(average)+ " max: "+ str(max(self.numvals["trans"]))
                            #average = sum(self.numvals["rot"]) / len(self.numvals["rot"])
                            #print "avg_rot: " + str(average) + " max: "+ str(max(self.numvals["rot"]))
        #plt.boxplot(
        #    self.numvals["trans"])
        #plt.hold(True)
        plt.figure(1)
        self.box_plt = plt.boxplot(
            self.numvals[str(num)]["trans"], positions=[int(num)], widths=0.6)
        plt.plot([int(num) - 0.3, int(num), int(num) + 0.3],
                 [self.numvals[str(num)]["avg_trans"], self.numvals[str(num)]["avg_trans"]
                     , self.numvals[str(num)]["avg_trans"]], color='g')

        if self.min_trans[1] > self.numvals[str(num)]["avg_trans"]:
            self.min_trans = [str(num), self.numvals[str(num)]["avg_trans"]]
        if self.min_med_trans[1] > self.box_plt['medians'][0].get_ydata()[0]:
            self.min_med_trans = [str(num), self.box_plt['medians'][0].get_ydata()[0]]
        #plt.hold(True)
        plt.figure(2)
        self.box_plt = plt.boxplot(
            self.numvals[str(num)]["rot"], positions=[int(num)], widths=0.6)
        plt.plot([int(num) - 0.3, int(num), int(num) + 0.3],
                 [self.numvals[str(num)]["avg_rot"], self.numvals[str(num)]["avg_rot"]
                     , self.numvals[str(num)]["avg_rot"]], color='g')

        if self.min_rot[1] > self.numvals[str(num)]["avg_rot"]:
            self.min_rot = [str(num), self.numvals[str(num)]["avg_rot"]]
        if self.min_med_rot[1] > self.box_plt['medians'][0].get_ydata()[0]:
            self.min_med_rot = [str(num), self.box_plt['medians'][0].get_ydata()[0]]
        self.pos = self.pos + 1

    def import_testnames(self, file):
        with open(file, 'r') as stream:
            # testlist = yaml.load(stream)
            # #print testlist
            # for test in testlist:
            #     for robot in test.iteritems():
            #         print "robot:", robot[1]['robot']
            #         self.testnames.append(robot[1]['robot'])
            self.testlist = yaml.load(stream)

    def show_results(self, single):
        if(single):
            print self.metric
            for metric in self.metric:
                plt.boxplot(self.numvals[metric]) #plt.bar(y_pos, means, yerr=devs, alpha=0.5, color='red')
                plt.show()
        else:
            #print "metric: ",self.metric
            counter = 0
            fig, axarr = plt.subplots(len(self.metric), sharex=True)
            for metric in self.metric:

                plt.title(metric)
                plt.boxplot(self.numvals[metric])
                #axarr[counter].text(xpos + width/2., 1.05*height,
                #        '%.1f' % round(height, 1),
                #        ha='center', va='bottom')
                counter += 1
            plt.tight_layout()
            plt.show()


    def calculate_data(self, metric):
        means = []
        devs = []
        testdata = []
        for testname in sorted(self.tests, key=lambda ts : int(ts.split('_')[2].replace('r', ''))): # Magic!
            data = self.tests[testname]
            # print "-------------------------------------------"
            # print testname
            # print sorted(self.tests, key=lambda ts : int(ts.split('_')[2].replace('r', '')))
            # print "\n data: ", data
            # print "mean: ",data[0][metric]
            # print "values: ", data[1][metric]
            means.extend(data[0][metric])
            #for mean in data[1][metric]:
            #print "dev", numpy.std(data[1][metric]), "from mean", data[1][metric]
            devs.append(numpy.std(data[1][metric]))
            testdata.append(data[1][metric])
            for test in self.testlist:
                #print "\n-------------------\ntest:", test, "\ntestname:", testname, "\n data: \n", data
                if testname in test:
                    #print "\n testname:", self.testlist, " \n \n test:", test
                    if (test[testname]['robot'] not in self.testnames):
                      self.testnames.append(test[testname]['robot'])
        # print("show")
        # print testnames
        # print self.metric
        # print plotdata
        y_pos = numpy.arange(len(self.testnames))
        # print "y pos", y_pos
        # print "height", means
        # print "deviation", devs
        return (y_pos, means, devs, testdata)

if __name__ == '__main__':
    parser = optparse.OptionParser()
    parser.add_option('-s', '--single', dest='single', help='Print all plots in single windows', default=False, action="store_true")
    parser.add_option('-p', '--path', dest='path', help='Target Path', default="~")
    (options, args) = parser.parse_args()

    p = presenter()
    Path = options.path
    filelist = os.listdir(Path)
    p.import_testnames(Path.replace('yaml', 'json')+"test_list.json")

    tick_counter = 0
    for file in filelist:
        #print "file", file
        if "merged" in str(file):
            p.import_yaml(Path+file)
            filename = file.replace('.yaml', '')
            pos = filename.find('_r')
            print pos
            end = filename.find('_', pos + 1)
            print end
            robot_num = filename[pos + 2 : end]
            print "robot: " + str(robot_num)
            #p.extract_yaml(filename.replace('merged_', ''))
            p.extract_yaml(robot_num)
            tick_counter = tick_counter + 1
    print "min mean trans: " + str(p.min_trans)
    print "min mean rot: " + str(p.min_rot)
    print "min median trans: " + str(p.min_med_trans)
    print "min median rot: " + str(p.min_med_rot)
    ticks = []
    ticks.append(-1)
    for i in xrange(tick_counter):
        ticks.append(i)
    ticks.append(tick_counter)
    plt.figure(1)
    plt.xticks(ticks, ticks)
    ax = plt.figure(1).add_subplot(111)
    ax.set_xlabel('Robot Number')
    ax.set_ylabel('delta trans in m')
    plt.figure(2)
    plt.xticks(ticks, ticks)
    ax = plt.figure(2).add_subplot(111)
    ax.set_xlabel('Robot Number')
    ax.set_ylabel('delta rot in degree')
    plt.show()

#p.show_results(options.single)