// tests/unit/HelloWorld.spec.js
import { mount } from '@vue/test-utils';
import HelloWorld from '@/components/HelloWorld.vue';

describe('HelloWorld.vue', () => {
    it('renders props.msg when passed', () => {
        const msg = 'Hello World';
        const wrapper = mount(HelloWorld, {
            props: { msg }
        });
        expect(wrapper.text()).toMatch(msg);
    });
});
