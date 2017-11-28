#!/usr/bin/env python

import StringIO
import csvquerytool
import rospkg

if __name__ == '__main__':
    rospack = rospkg.RosPack()
    # filename = rospack.get_path('test') + '/log/1StageResults.csv'
    filename = '/home/ninja/indigo_ws/src/space_robotics_challenge/val_com/test/log/1StageResults.csv'
    stdout = StringIO.StringIO()
    data = csvquerytool.run_query('SELECT "position.x", "position.y", count("CoM.x_ddot") as successful_runs FROM csv where "position.x2" = 1 AND'
                                  '"CoM.x_ddot" = "true" group by "position.x", "position.y" order by "position.x" asc, "position.y" desc',
                                  [filename], stdout)
    data = stdout.getvalue()
    print data


# select success.x, success.y, s1/(s1+s2)*100  from
    # (select x, y , count(success) as s1 from csv where success="true" and z = 1
    #  group by x, y order by x asc, y desc) success,
    # (select x, y , count(success) as s2 from csv where and z = 1
    #  group by x, y order by x asc, y desc) total
    #  where success.x = total.x and success.y = total.y
    # group by success.x, success.y