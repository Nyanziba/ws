from example_interfaces.srv import TextSplit

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        # サービスを作成し，タイプ，名前，およびコールバックを定義．
        self.srv = self.create_service(TextSplit, 'text_split', self.text_split_callback)
        # String reqest


    def text_split_callback(self, req, res1, res2):
        input_text = req.reqest
        res1,res2 = self.split_text(input_text)
        if res1 == False or res2 == False:
            res1 = "False"
            res2 = "False"
        return res1, res2

    def split_text(self, text):
        person_name = ["Adam","Axel","Chris","Hunter","Jack","Max","Paris","Robin","Olivia","William"]
        favarite_drink = ["coke","green tea","orange juice","soda","wine","water"]
        #これらは2024ルールの正式なリストです、これらの中からランダムに選択されます。
        #方針はこれらの単語が含まれているかどうかを判定、どちらも含まれていたら二つをresponseし、片方しか聞き取れていなかったらfalseを出して聞き直します。
        input_text = text.split()
        person_nm = "False"
        favarite_drk = "False"
        for i  in person_name:
            if input_text in person_name[i]:
                person_nm = person_name[i]
                continue

        for h  in favarite_drink:
            if input_text in favarite_drink[h]:
                favarite_drk = favarite_drink[h]
                continue

        return person_nm, favarite_drk


def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
